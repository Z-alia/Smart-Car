/**
 * @file trajectory_flash_storage.c
 * @brief 惯导轨迹数据Flash存储模块实现
 * @note 基于STM32 HAL QSPI驱动，支持W25Q系列Flash
 * @date 2025-11-20
 */

#include "trajectory_flash_storage.h"
#include "quadspi.h"
#include <string.h>

// ==================== W25Q Flash命令定义 ====================

#define W25Q_CMD_WRITE_ENABLE       0x06    // 写使能
#define W25Q_CMD_WRITE_DISABLE      0x04    // 写禁止
#define W25Q_CMD_READ_STATUS_REG1   0x05    // 读状态寄存器1
#define W25Q_CMD_READ_STATUS_REG2   0x35    // 读状态寄存器2
#define W25Q_CMD_WRITE_STATUS_REG   0x01    // 写状态寄存器
#define W25Q_CMD_PAGE_PROGRAM       0x02    // 页编程
#define W25Q_CMD_QUAD_PAGE_PROGRAM  0x32    // 四线页编程
#define W25Q_CMD_SECTOR_ERASE       0x20    // 扇区擦除 (4KB)
#define W25Q_CMD_BLOCK_ERASE_32K    0x52    // 块擦除 (32KB)
#define W25Q_CMD_BLOCK_ERASE_64K    0xD8    // 块擦除 (64KB)
#define W25Q_CMD_CHIP_ERASE         0xC7    // 芯片擦除
#define W25Q_CMD_READ_DATA          0x03    // 读数据
#define W25Q_CMD_FAST_READ          0x0B    // 快速读
#define W25Q_CMD_READ_ID            0x9F    // 读取ID

#define W25Q_STATUS_BUSY            0x01    // 忙标志位
#define W25Q_TIMEOUT_MS             5000    // 超时时间

// ==================== 全局实例定义 ====================

FlashStorageManager_t g_flash_storage = {0};

// ==================== CRC32校验 ====================

/**
 * @brief CRC32计算 (使用STM32硬件CRC或软件实现)
 */
static uint32_t calculate_crc32(const uint8_t* data, uint32_t length)
{
    // 简单的软件CRC32实现 (也可以使用STM32的硬件CRC)
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}

// ==================== 底层Flash操作 ====================

/**
 * @brief 等待Flash操作完成
 */
static FlashStorageStatus_t flash_wait_ready(uint32_t timeout_ms)
{
    QSPI_CommandTypeDef cmd = {0};
    uint8_t status;
    uint32_t start_tick = HAL_GetTick();
    
    // 配置读状态寄存器命令
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction = W25Q_CMD_READ_STATUS_REG1;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = 1;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    
    // 轮询等待忙标志清除
    do {
        if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return FLASH_STORAGE_READ_FAILED;
        }
        
        if (HAL_QSPI_Receive(&hqspi, &status, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return FLASH_STORAGE_READ_FAILED;
        }
        
        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            return FLASH_STORAGE_INIT_FAILED;
        }
        
    } while (status & W25Q_STATUS_BUSY);
    
    return FLASH_STORAGE_OK;
}

/**
 * @brief 发送写使能命令
 */
static FlashStorageStatus_t flash_write_enable(void)
{
    QSPI_CommandTypeDef cmd = {0};
    
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction = W25Q_CMD_WRITE_ENABLE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_STORAGE_WRITE_FAILED;
    }
    
    return FLASH_STORAGE_OK;
}

/**
 * @brief Flash读取
 */
FlashStorageStatus_t flash_read(uint32_t address, uint8_t* buffer, uint32_t size)
{
    QSPI_CommandTypeDef cmd = {0};
    
    // 配置读命令
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction = W25Q_CMD_READ_DATA;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.Address = address;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = size;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_STORAGE_READ_FAILED;
    }
    
    if (HAL_QSPI_Receive(&hqspi, buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_STORAGE_READ_FAILED;
    }
    
    return FLASH_STORAGE_OK;
}

/**
 * @brief Flash页写入 (最大256字节)
 */
static FlashStorageStatus_t flash_write_page(uint32_t address, const uint8_t* buffer, uint32_t size)
{
    QSPI_CommandTypeDef cmd = {0};
    FlashStorageStatus_t status;
    
    // 等待Flash准备好
    status = flash_wait_ready(W25Q_TIMEOUT_MS);
    if (status != FLASH_STORAGE_OK) return status;
    
    // 写使能
    status = flash_write_enable();
    if (status != FLASH_STORAGE_OK) return status;
    
    // 配置页编程命令
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction = W25Q_CMD_PAGE_PROGRAM;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.Address = address;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = size;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_STORAGE_WRITE_FAILED;
    }
    
    if (HAL_QSPI_Transmit(&hqspi, (uint8_t*)buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_STORAGE_WRITE_FAILED;
    }
    
    // 等待写入完成
    return flash_wait_ready(W25Q_TIMEOUT_MS);
}

/**
 * @brief Flash写入 (自动分页)
 */
FlashStorageStatus_t flash_write(uint32_t address, const uint8_t* buffer, uint32_t size)
{
    uint32_t remaining = size;
    uint32_t current_addr = address;
    const uint8_t* current_data = buffer;
    FlashStorageStatus_t status;
    
    while (remaining > 0) {
        // 计算当前页可写字节数
        uint32_t page_offset = current_addr % FLASH_PAGE_SIZE;
        uint32_t page_remaining = FLASH_PAGE_SIZE - page_offset;
        uint32_t write_size = (remaining < page_remaining) ? remaining : page_remaining;
        
        // 写入当前页
        status = flash_write_page(current_addr, current_data, write_size);
        if (status != FLASH_STORAGE_OK) {
            return status;
        }
        
        // 更新指针
        current_addr += write_size;
        current_data += write_size;
        remaining -= write_size;
    }
    
    return FLASH_STORAGE_OK;
}

/**
 * @brief Flash扇区擦除
 */
FlashStorageStatus_t flash_erase_sector(uint32_t address)
{
    QSPI_CommandTypeDef cmd = {0};
    FlashStorageStatus_t status;
    
    // 等待Flash准备好
    status = flash_wait_ready(W25Q_TIMEOUT_MS);
    if (status != FLASH_STORAGE_OK) return status;
    
    // 写使能
    status = flash_write_enable();
    if (status != FLASH_STORAGE_OK) return status;
    
    // 配置扇区擦除命令
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction = W25Q_CMD_SECTOR_ERASE;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.Address = address;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_STORAGE_ERASE_FAILED;
    }
    
    // 等待擦除完成
    return flash_wait_ready(W25Q_TIMEOUT_MS);
}

/**
 * @brief Flash芯片擦除
 */
FlashStorageStatus_t flash_erase_chip(void)
{
    QSPI_CommandTypeDef cmd = {0};
    FlashStorageStatus_t status;
    
    status = flash_wait_ready(W25Q_TIMEOUT_MS);
    if (status != FLASH_STORAGE_OK) return status;
    
    status = flash_write_enable();
    if (status != FLASH_STORAGE_OK) return status;
    
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction = W25Q_CMD_CHIP_ERASE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return FLASH_STORAGE_ERASE_FAILED;
    }
    
    return flash_wait_ready(30000);  // 芯片擦除需要更长时间
}

// ==================== 高层存储管理 ====================

/**
 * @brief 初始化Flash存储模块
 */
FlashStorageStatus_t trajectory_flash_init(void)
{
    FlashStorageStatus_t status;
    
    // 等待Flash准备好
    status = flash_wait_ready(W25Q_TIMEOUT_MS);
    if (status != FLASH_STORAGE_OK) {
        g_flash_storage.last_error = status;
        return status;
    }
    
    // 读取索引表
    status = flash_read(FLASH_STORAGE_START_ADDR, 
                       (uint8_t*)&g_flash_storage.index, 
                       sizeof(TrajectoryIndex_t));
    if (status != FLASH_STORAGE_OK) {
        g_flash_storage.last_error = status;
        return status;
    }
    
    // 检查魔数
    if (g_flash_storage.index.magic != FLASH_STORAGE_MAGIC) {
        // Flash为空或数据损坏，创建新索引
        memset(&g_flash_storage.index, 0, sizeof(TrajectoryIndex_t));
        g_flash_storage.index.magic = FLASH_STORAGE_MAGIC;
        g_flash_storage.index.version = FLASH_STORAGE_VERSION;
        g_flash_storage.index.trajectory_count = 0;
        
        // 写入新索引到Flash
        status = flash_erase_sector(FLASH_STORAGE_START_ADDR);
        if (status != FLASH_STORAGE_OK) {
            g_flash_storage.last_error = status;
            return status;
        }
        
        // 计算并写入CRC
        g_flash_storage.index.index_crc32 = calculate_crc32(
            (uint8_t*)&g_flash_storage.index,
            sizeof(TrajectoryIndex_t) - sizeof(uint32_t)
        );
        
        status = flash_write(FLASH_STORAGE_START_ADDR,
                            (uint8_t*)&g_flash_storage.index,
                            sizeof(TrajectoryIndex_t));
        if (status != FLASH_STORAGE_OK) {
            g_flash_storage.last_error = status;
            return status;
        }
    } else {
        // 验证CRC
        uint32_t calculated_crc = calculate_crc32(
            (uint8_t*)&g_flash_storage.index,
            sizeof(TrajectoryIndex_t) - sizeof(uint32_t)
        );
        
        if (calculated_crc != g_flash_storage.index.index_crc32) {
            g_flash_storage.last_error = FLASH_STORAGE_CRC_ERROR;
            return FLASH_STORAGE_CRC_ERROR;
        }
    }
    
    g_flash_storage.is_initialized = 1;
    g_flash_storage.last_error = FLASH_STORAGE_OK;
    return FLASH_STORAGE_OK;
}

/**
 * @brief 计算轨迹在Flash中的地址
 */
static uint32_t get_trajectory_address(uint8_t trajectory_id)
{
    return FLASH_STORAGE_START_ADDR + FLASH_SECTOR_SIZE + 
           (trajectory_id * FLASH_TRAJECTORY_SIZE);
}

/**
 * @brief 保存轨迹到Flash
 */
FlashStorageStatus_t trajectory_flash_save(uint8_t trajectory_id, 
                                           const char* name, 
                                           const char* description)
{
    if (!g_flash_storage.is_initialized) {
        return FLASH_STORAGE_INIT_FAILED;
    }
    
    if (trajectory_id >= FLASH_MAX_TRAJECTORIES) {
        return FLASH_STORAGE_INVALID_PARAM;
    }
    
    // 获取惯导系统数据
    InertialNav_t* nav = inertial_nav_get_state();
    if (nav->trajectory.count == 0) {
        return FLASH_STORAGE_INVALID_PARAM;
    }
    
    // 准备元数据
    TrajectoryMetadata_t metadata = {0};
    metadata.magic = FLASH_STORAGE_MAGIC;
    metadata.version = FLASH_STORAGE_VERSION;
    metadata.trajectory_id = trajectory_id;
    metadata.waypoint_count = nav->trajectory.count;
    metadata.timestamp = HAL_GetTick();
    
    // 计算轨迹信息
    inertial_nav_get_trajectory_info(NULL, &metadata.duration_ms, &metadata.total_distance);
    
    // 起点和终点
    Pose_t pose;
    decompress_waypoint(&nav->trajectory.points[0], &pose);
    metadata.start_x = pose.x;
    metadata.start_y = pose.y;
    metadata.start_theta = pose.theta;
    
    decompress_waypoint(&nav->trajectory.points[nav->trajectory.count-1], &pose);
    metadata.end_x = pose.x;
    metadata.end_y = pose.y;
    metadata.end_theta = pose.theta;
    
    // 复制名称和描述
    if (name) {
        strncpy((char*)metadata.name, name, sizeof(metadata.name) - 1);
    }
    if (description) {
        strncpy((char*)metadata.description, description, sizeof(metadata.description) - 1);
    }
    
    // 计算航点数据CRC
    metadata.data_crc32 = calculate_crc32(
        (uint8_t*)nav->trajectory.points,
        nav->trajectory.count * sizeof(Waypoint_t)
    );
    
    // 计算元数据CRC
    metadata.header_crc32 = calculate_crc32(
        (uint8_t*)&metadata,
        sizeof(TrajectoryMetadata_t) - sizeof(uint32_t)
    );
    
    // 计算Flash地址
    uint32_t traj_addr = get_trajectory_address(trajectory_id);
    
    // 擦除扇区
    for (uint32_t i = 0; i < 6; i++) {  // 6个扇区
        FlashStorageStatus_t status = flash_erase_sector(traj_addr + i * FLASH_SECTOR_SIZE);
        if (status != FLASH_STORAGE_OK) {
            g_flash_storage.last_error = status;
            return status;
        }
    }
    
    // 写入元数据
    FlashStorageStatus_t status = flash_write(traj_addr, 
                                              (uint8_t*)&metadata,
                                              sizeof(TrajectoryMetadata_t));
    if (status != FLASH_STORAGE_OK) {
        g_flash_storage.last_error = status;
        return status;
    }
    
    // 写入航点数据
    status = flash_write(traj_addr + sizeof(TrajectoryMetadata_t),
                        (uint8_t*)nav->trajectory.points,
                        nav->trajectory.count * sizeof(Waypoint_t));
    if (status != FLASH_STORAGE_OK) {
        g_flash_storage.last_error = status;
        return status;
    }
    
    // 更新索引表
    uint8_t found = 0;
    for (uint16_t i = 0; i < g_flash_storage.index.trajectory_count; i++) {
        if (g_flash_storage.index.trajectories[i].trajectory_id == trajectory_id) {
            g_flash_storage.index.trajectories[i] = metadata;
            found = 1;
            break;
        }
    }
    
    if (!found && g_flash_storage.index.trajectory_count < FLASH_MAX_TRAJECTORIES) {
        g_flash_storage.index.trajectories[g_flash_storage.index.trajectory_count] = metadata;
        g_flash_storage.index.trajectory_count++;
    }
    
    // 写回索引表
    g_flash_storage.index.index_crc32 = calculate_crc32(
        (uint8_t*)&g_flash_storage.index,
        sizeof(TrajectoryIndex_t) - sizeof(uint32_t)
    );
    
    status = flash_erase_sector(FLASH_STORAGE_START_ADDR);
    if (status != FLASH_STORAGE_OK) {
        g_flash_storage.last_error = status;
        return status;
    }
    
    status = flash_write(FLASH_STORAGE_START_ADDR,
                        (uint8_t*)&g_flash_storage.index,
                        sizeof(TrajectoryIndex_t));
    
    g_flash_storage.last_error = status;
    return status;
}

/**
 * @brief 从Flash加载轨迹
 */
FlashStorageStatus_t trajectory_flash_load(uint8_t trajectory_id)
{
    if (!g_flash_storage.is_initialized) {
        return FLASH_STORAGE_INIT_FAILED;
    }
    
    if (trajectory_id >= FLASH_MAX_TRAJECTORIES) {
        return FLASH_STORAGE_INVALID_PARAM;
    }
    
    // 计算地址
    uint32_t traj_addr = get_trajectory_address(trajectory_id);
    
    // 读取元数据
    TrajectoryMetadata_t metadata;
    FlashStorageStatus_t status = flash_read(traj_addr, 
                                             (uint8_t*)&metadata,
                                             sizeof(TrajectoryMetadata_t));
    if (status != FLASH_STORAGE_OK) {
        g_flash_storage.last_error = status;
        return status;
    }
    
    // 验证魔数
    if (metadata.magic != FLASH_STORAGE_MAGIC) {
        g_flash_storage.last_error = FLASH_STORAGE_NOT_FOUND;
        return FLASH_STORAGE_NOT_FOUND;
    }
    
    // 验证元数据CRC
    uint32_t calc_crc = calculate_crc32((uint8_t*)&metadata,
                                       sizeof(TrajectoryMetadata_t) - sizeof(uint32_t));
    if (calc_crc != metadata.header_crc32) {
        g_flash_storage.last_error = FLASH_STORAGE_CRC_ERROR;
        return FLASH_STORAGE_CRC_ERROR;
    }
    
    // 读取航点数据
    InertialNav_t* nav = inertial_nav_get_state();
    status = flash_read(traj_addr + sizeof(TrajectoryMetadata_t),
                       (uint8_t*)nav->trajectory.points,
                       metadata.waypoint_count * sizeof(Waypoint_t));
    if (status != FLASH_STORAGE_OK) {
        g_flash_storage.last_error = status;
        return status;
    }
    
    // 验证航点数据CRC
    calc_crc = calculate_crc32((uint8_t*)nav->trajectory.points,
                              metadata.waypoint_count * sizeof(Waypoint_t));
    if (calc_crc != metadata.data_crc32) {
        g_flash_storage.last_error = FLASH_STORAGE_CRC_ERROR;
        return FLASH_STORAGE_CRC_ERROR;
    }
    
    // 更新轨迹状态
    nav->trajectory.count = metadata.waypoint_count;
    nav->trajectory.current_index = 0;
    nav->trajectory.is_recording = 0;
    nav->trajectory.is_replaying = 0;
    
    g_flash_storage.last_error = FLASH_STORAGE_OK;
    return FLASH_STORAGE_OK;
}

/**
 * @brief 删除轨迹
 */
FlashStorageStatus_t trajectory_flash_delete(uint8_t trajectory_id)
{
    if (!g_flash_storage.is_initialized) {
        return FLASH_STORAGE_INIT_FAILED;
    }
    
    // 从索引表中移除
    uint8_t found = 0;
    for (uint16_t i = 0; i < g_flash_storage.index.trajectory_count; i++) {
        if (g_flash_storage.index.trajectories[i].trajectory_id == trajectory_id) {
            // 移动后续元素
            for (uint16_t j = i; j < g_flash_storage.index.trajectory_count - 1; j++) {
                g_flash_storage.index.trajectories[j] = g_flash_storage.index.trajectories[j + 1];
            }
            g_flash_storage.index.trajectory_count--;
            found = 1;
            break;
        }
    }
    
    if (!found) {
        return FLASH_STORAGE_NOT_FOUND;
    }
    
    // 擦除Flash区域
    uint32_t traj_addr = get_trajectory_address(trajectory_id);
    for (uint32_t i = 0; i < 6; i++) {
        FlashStorageStatus_t status = flash_erase_sector(traj_addr + i * FLASH_SECTOR_SIZE);
        if (status != FLASH_STORAGE_OK) {
            g_flash_storage.last_error = status;
            return status;
        }
    }
    
    // 更新索引表
    g_flash_storage.index.index_crc32 = calculate_crc32(
        (uint8_t*)&g_flash_storage.index,
        sizeof(TrajectoryIndex_t) - sizeof(uint32_t)
    );
    
    FlashStorageStatus_t status = flash_erase_sector(FLASH_STORAGE_START_ADDR);
    if (status != FLASH_STORAGE_OK) {
        g_flash_storage.last_error = status;
        return status;
    }
    
    status = flash_write(FLASH_STORAGE_START_ADDR,
                        (uint8_t*)&g_flash_storage.index,
                        sizeof(TrajectoryIndex_t));
    
    g_flash_storage.last_error = status;
    return status;
}

/**
 * @brief 格式化Flash
 */
FlashStorageStatus_t trajectory_flash_format(void)
{
    // 清空索引
    memset(&g_flash_storage.index, 0, sizeof(TrajectoryIndex_t));
    g_flash_storage.index.magic = FLASH_STORAGE_MAGIC;
    g_flash_storage.index.version = FLASH_STORAGE_VERSION;
    
    // 重新初始化
    return trajectory_flash_init();
}

/**
 * @brief 获取轨迹列表
 */
FlashStorageStatus_t trajectory_flash_get_list(TrajectoryMetadata_t** list, uint16_t* count)
{
    if (!g_flash_storage.is_initialized) {
        return FLASH_STORAGE_INIT_FAILED;
    }
    
    *list = g_flash_storage.index.trajectories;
    *count = g_flash_storage.index.trajectory_count;
    
    return FLASH_STORAGE_OK;
}

/**
 * @brief 检查轨迹是否存在
 */
uint8_t trajectory_flash_exists(uint8_t trajectory_id)
{
    for (uint16_t i = 0; i < g_flash_storage.index.trajectory_count; i++) {
        if (g_flash_storage.index.trajectories[i].trajectory_id == trajectory_id) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 获取错误描述
 */
const char* trajectory_flash_get_error_string(FlashStorageStatus_t status)
{
    static const char* error_strings[] = {
        "OK",
        "Init Failed",
        "Write Failed",
        "Read Failed",
        "Erase Failed",
        "CRC Error",
        "Storage Full",
        "Not Found",
        "Invalid Parameter"
    };
    
    if (status < sizeof(error_strings) / sizeof(error_strings[0])) {
        return error_strings[status];
    }
    return "Unknown Error";
}

FlashStorageStatus_t trajectory_flash_get_last_error(void)
{
    return g_flash_storage.last_error;
}
