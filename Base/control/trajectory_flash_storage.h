/**
 * @file trajectory_flash_storage.h
 * @brief 惯导轨迹数据Flash存储模块 (掉电保持)
 * @note 功能:
 *       1. 将惯导轨迹数据保存到外部QSPI Flash
 *       2. 掉电后数据仍然存在
 *       3. 支持多条轨迹存储和管理
 *       4. 数据完整性校验 (CRC32)
 * @date 2025-11-20
 * @hardware STM32H750 + W25Q64/W25Q128 (QSPI Flash)
 */

#ifndef TRAJECTORY_FLASH_STORAGE_H_
#define TRAJECTORY_FLASH_STORAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "inertial_nav.h"

// ==================== Flash配置参数 ====================

#define FLASH_STORAGE_MAGIC         0x54524143  // "TRAC" 魔数标识
#define FLASH_STORAGE_VERSION       0x0100      // 版本号 v1.0

// Flash扇区配置 (根据实际Flash型号调整)
#define FLASH_SECTOR_SIZE           4096        // 扇区大小 4KB
#define FLASH_PAGE_SIZE             256         // 页大小 256B
#define FLASH_STORAGE_START_ADDR    0x00000000  // 存储起始地址
#define FLASH_MAX_TRAJECTORIES      8           // 最大轨迹数量

// 单条轨迹占用空间计算
// 航点数据: 3000 × 8 = 24000 bytes
// 元数据: ~256 bytes
// 总计: ~24KB (6个扇区)
#define FLASH_TRAJECTORY_SIZE       (6 * FLASH_SECTOR_SIZE)  // 24KB/轨迹

// ==================== 数据结构定义 ====================

/**
 * @brief 轨迹元数据 (存储在Flash)
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;             // 魔数 0x54524143 ("TRAC")
    uint16_t version;           // 版本号
    uint16_t trajectory_id;     // 轨迹ID (0~7)
    
    uint16_t waypoint_count;    // 航点数量
    uint16_t reserved1;         // 保留
    
    uint32_t timestamp;         // 录制时间戳 (Unix时间或系统运行时间)
    float total_distance;       // 轨迹总长度 (m)
    uint32_t duration_ms;       // 轨迹总时长 (ms)
    
    float start_x;              // 起点X坐标
    float start_y;              // 起点Y坐标
    float start_theta;          // 起点航向角
    
    float end_x;                // 终点X坐标
    float end_y;                // 终点Y坐标
    float end_theta;            // 终点航向角
    
    uint32_t data_crc32;        // 航点数据CRC32校验
    uint32_t header_crc32;      // 元数据CRC32校验
    
    uint8_t name[32];           // 轨迹名称 (可选)
    uint8_t description[64];    // 轨迹描述 (可选)
    
    uint8_t reserved2[64];      // 保留扩展字段
    
} TrajectoryMetadata_t;  // 总大小: 256 bytes

/**
 * @brief 轨迹索引表 (存储在Flash首个扇区)
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;                         // 魔数
    uint16_t version;                       // 版本号
    uint16_t trajectory_count;              // 已存储的轨迹数量
    
    TrajectoryMetadata_t trajectories[FLASH_MAX_TRAJECTORIES];  // 轨迹元数据
    
    uint32_t index_crc32;                   // 索引表CRC32校验
    
} TrajectoryIndex_t;

/**
 * @brief Flash存储状态
 */
typedef enum {
    FLASH_STORAGE_OK = 0,           // 正常
    FLASH_STORAGE_INIT_FAILED,      // 初始化失败
    FLASH_STORAGE_WRITE_FAILED,     // 写入失败
    FLASH_STORAGE_READ_FAILED,      // 读取失败
    FLASH_STORAGE_ERASE_FAILED,     // 擦除失败
    FLASH_STORAGE_CRC_ERROR,        // CRC校验错误
    FLASH_STORAGE_FULL,             // 存储空间已满
    FLASH_STORAGE_NOT_FOUND,        // 轨迹未找到
    FLASH_STORAGE_INVALID_PARAM,    // 参数错误
} FlashStorageStatus_t;

/**
 * @brief Flash存储管理器
 */
typedef struct {
    uint8_t is_initialized;         // 初始化标志
    TrajectoryIndex_t index;        // 索引表 (缓存在RAM)
    uint8_t current_trajectory_id;  // 当前操作的轨迹ID
    FlashStorageStatus_t last_error;// 最后错误码
} FlashStorageManager_t;

// ==================== 全局实例 ====================

extern FlashStorageManager_t g_flash_storage;

// ==================== 核心接口函数 ====================

/**
 * @brief 初始化Flash存储模块
 * @return FlashStorageStatus_t 状态码
 * @note 必须先调用 MX_QUADSPI_Init() 初始化QSPI硬件
 * @note 会读取索引表，如果Flash为空会创建新索引
 */
FlashStorageStatus_t trajectory_flash_init(void);

/**
 * @brief 保存当前惯导轨迹到Flash
 * @param trajectory_id 轨迹ID (0~7)
 * @param name 轨迹名称 (可选, NULL使用默认名称)
 * @param description 轨迹描述 (可选, NULL为空)
 * @return FlashStorageStatus_t 状态码
 * @note 会自动从 g_inertial_nav.trajectory 读取数据
 * @note 如果轨迹ID已存在，会覆盖旧数据
 * @note 使用示例:
 *       // 录制完成后保存
 *       inertial_nav_stop_recording();
 *       trajectory_flash_save(0, "Obstacle Avoidance", "Recorded at 2025-11-20");
 */
FlashStorageStatus_t trajectory_flash_save(uint8_t trajectory_id, 
                                           const char* name, 
                                           const char* description);

/**
 * @brief 从Flash加载轨迹到惯导系统
 * @param trajectory_id 轨迹ID (0~7)
 * @return FlashStorageStatus_t 状态码
 * @note 会自动加载到 g_inertial_nav.trajectory
 * @note 加载后可直接调用 inertial_nav_start_replay() 复现
 * @note 使用示例:
 *       // 启动前加载轨迹
 *       trajectory_flash_load(0);
 *       inertial_nav_start_replay();
 */
FlashStorageStatus_t trajectory_flash_load(uint8_t trajectory_id);

/**
 * @brief 删除Flash中的轨迹
 * @param trajectory_id 轨迹ID (0~7)
 * @return FlashStorageStatus_t 状态码
 */
FlashStorageStatus_t trajectory_flash_delete(uint8_t trajectory_id);

/**
 * @brief 格式化Flash存储区 (清除所有轨迹)
 * @return FlashStorageStatus_t 状态码
 * @note 危险操作！会删除所有已保存的轨迹
 */
FlashStorageStatus_t trajectory_flash_format(void);

/**
 * @brief 获取轨迹列表
 * @param list 输出轨迹元数据数组指针
 * @param count 输出轨迹数量
 * @return FlashStorageStatus_t 状态码
 * @note 使用示例:
 *       TrajectoryMetadata_t* list;
 *       uint16_t count;
 *       trajectory_flash_get_list(&list, &count);
 *       for (int i = 0; i < count; i++) {
 *           printf("ID: %d, Name: %s, Points: %d\n",
 *                  list[i].trajectory_id,
 *                  list[i].name,
 *                  list[i].waypoint_count);
 *       }
 */
FlashStorageStatus_t trajectory_flash_get_list(TrajectoryMetadata_t** list, 
                                               uint16_t* count);

/**
 * @brief 获取轨迹元数据
 * @param trajectory_id 轨迹ID
 * @param metadata 输出元数据指针
 * @return FlashStorageStatus_t 状态码
 */
FlashStorageStatus_t trajectory_flash_get_metadata(uint8_t trajectory_id,
                                                   TrajectoryMetadata_t* metadata);

/**
 * @brief 检查轨迹是否存在
 * @param trajectory_id 轨迹ID
 * @return 1=存在, 0=不存在
 */
uint8_t trajectory_flash_exists(uint8_t trajectory_id);

/**
 * @brief 获取Flash存储使用情况
 * @param used_trajectories 已使用轨迹槽位数
 * @param total_trajectories 总轨迹槽位数
 * @param used_bytes 已使用字节数
 * @param total_bytes 总字节数
 */
void trajectory_flash_get_usage(uint16_t* used_trajectories,
                                uint16_t* total_trajectories,
                                uint32_t* used_bytes,
                                uint32_t* total_bytes);

/**
 * @brief 获取最后错误码
 * @return FlashStorageStatus_t 错误码
 */
FlashStorageStatus_t trajectory_flash_get_last_error(void);

/**
 * @brief 获取错误描述字符串
 * @param status 状态码
 * @return 错误描述字符串
 */
const char* trajectory_flash_get_error_string(FlashStorageStatus_t status);

// ==================== 高级功能 ====================

/**
 * @brief 导出轨迹数据 (通过串口或USB)
 * @param trajectory_id 轨迹ID
 * @param callback 回调函数，每个航点调用一次
 * @return FlashStorageStatus_t 状态码
 * @note 使用示例:
 *       void export_callback(const Waypoint_t* wp, uint16_t index) {
 *           printf("%d,%.3f,%.3f,%.3f,%.3f\n", 
 *                  index, 
 *                  wp->x/1000.0f, wp->y/1000.0f, 
 *                  wp->theta/100.0f, wp->v/1000.0f);
 *       }
 *       trajectory_flash_export(0, export_callback);
 */
FlashStorageStatus_t trajectory_flash_export(uint8_t trajectory_id,
                                             void (*callback)(const Waypoint_t* wp, uint16_t index));

/**
 * @brief 导入轨迹数据 (从串口或文件)
 * @param trajectory_id 目标轨迹ID
 * @param waypoints 航点数组
 * @param count 航点数量
 * @param name 轨迹名称 (可选)
 * @param description 轨迹描述 (可选)
 * @return FlashStorageStatus_t 状态码
 */
FlashStorageStatus_t trajectory_flash_import(uint8_t trajectory_id,
                                             const Waypoint_t* waypoints,
                                             uint16_t count,
                                             const char* name,
                                             const char* description);

/**
 * @brief 验证Flash数据完整性
 * @param trajectory_id 轨迹ID (0xFF表示验证所有)
 * @return FlashStorageStatus_t 状态码
 */
FlashStorageStatus_t trajectory_flash_verify(uint8_t trajectory_id);

/**
 * @brief 备份轨迹到另一个槽位
 * @param src_id 源轨迹ID
 * @param dst_id 目标轨迹ID
 * @return FlashStorageStatus_t 状态码
 */
FlashStorageStatus_t trajectory_flash_backup(uint8_t src_id, uint8_t dst_id);

// ==================== 底层Flash操作接口 ====================

/**
 * @brief 底层Flash读取
 * @param address Flash地址
 * @param buffer 数据缓冲区
 * @param size 读取字节数
 * @return FlashStorageStatus_t 状态码
 * @note 内部使用，一般不需要直接调用
 */
FlashStorageStatus_t flash_read(uint32_t address, uint8_t* buffer, uint32_t size);

/**
 * @brief 底层Flash写入
 * @param address Flash地址
 * @param buffer 数据缓冲区
 * @param size 写入字节数
 * @return FlashStorageStatus_t 状态码
 */
FlashStorageStatus_t flash_write(uint32_t address, const uint8_t* buffer, uint32_t size);

/**
 * @brief 底层Flash扇区擦除
 * @param address 扇区地址
 * @return FlashStorageStatus_t 状态码
 */
FlashStorageStatus_t flash_erase_sector(uint32_t address);

/**
 * @brief 底层Flash芯片擦除
 * @return FlashStorageStatus_t 状态码
 * @note 危险操作！会擦除整个Flash芯片
 */
FlashStorageStatus_t flash_erase_chip(void);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_FLASH_STORAGE_H_ */

/*
================================================================================
                            使用流程说明
================================================================================

1. 初始化
   MX_QUADSPI_Init();           // HAL QSPI初始化
   trajectory_flash_init();     // Flash存储模块初始化

2. 录制并保存轨迹
   inertial_nav_start_recording();
   // ... 录制过程 ...
   inertial_nav_stop_recording();
   trajectory_flash_save(0, "Obstacle Route", "First recording");

3. 掉电重启后加载轨迹
   trajectory_flash_init();     // 重新初始化
   trajectory_flash_load(0);    // 加载轨迹0
   inertial_nav_start_replay(); // 开始复现

4. 查看所有轨迹
   TrajectoryMetadata_t* list;
   uint16_t count;
   trajectory_flash_get_list(&list, &count);
   for (int i = 0; i < count; i++) {
       printf("ID: %d, Name: %s, Points: %d\n",
              list[i].trajectory_id, list[i].name, list[i].waypoint_count);
   }

================================================================================
                            Flash空间规划
================================================================================

Flash总容量: 8MB (W25Q64)
使用空间: 192KB (8条轨迹 × 24KB)

地址分配:
  0x000000 - 0x000FFF : 索引表 (4KB, 1个扇区)
  0x001000 - 0x006FFF : 轨迹0 (24KB, 6个扇区)
  0x007000 - 0x00CFFF : 轨迹1 (24KB, 6个扇区)
  0x00D000 - 0x012FFF : 轨迹2 (24KB, 6个扇区)
  ...
  0x031000 - 0x036FFF : 轨迹7 (24KB, 6个扇区)

预留空间: 剩余7.8MB可用于其他数据

================================================================================
*/
