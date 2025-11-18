/**
 ******************************************************************************
 * @file    LQ_Transfer_Image_STM32.c
 * @brief   WiFi图传模块驱动(STM32H750移植版本)
 * @note    基于SPI协议的WiFi图传模块,支持灰度图和二值化图像传输
 ******************************************************************************
 */

#include "LQ_Transfer_Image.h"

/* 数据包头尾标识 */
unsigned char FH[4] = {0xa0, 0xff, 0xff, 0xa0};
unsigned char FE[4] = {0xb0, 0xb0, 0x0a, 0x0d};

/**
 * @brief  微秒级延时函数
 * @param  us: 延时微秒数
 * @note   基于HAL_RCC_GetHCLKFreq()实现精确延时
 *         STM32H750默认480MHz主频，可根据实际配置调整
 */
void delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;
    
    ticks = us * (SystemCoreClock / 1000000);  // 计算需要的tick数
    told = SysTick->VAL;
    
    while(1)
    {
        tnow = SysTick->VAL;
        if(tnow != told)
        {
            if(tnow < told)
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if(tcnt >= ticks)
                break;
        }
    }
}

/**
 * @brief  WiFi图传模块初始化
 * @note   初始化SPI3接口和相关GPIO
 *         - CS引脚: PD13 (软件控制片选)
 *         - IO1引脚: PD11 (模式配置,拉高进入WiFi模式)
 *         - IO2引脚: PB6 (握手信号)
 *         - SPI3: SCK-PC10, MISO-PC11, MOSI-PB2
 */
void TR_driver_init(void)
{
    /* SPI3已经在MX_SPI3_Init()中初始化 */
    
    /* GPIO引脚已经在MX_GPIO_Init()中配置:
     * - WIFI_CS (PD13): 输出模式，默认高电平
     * - WIFI_IO1 (PD11): 输出模式，默认高电平
     * - WIFI_IO2 (PB6): 输入模式，无上下拉
     */
    
    /* 设置WiFi模块工作模式 */
    TR_IO1_H;        // IO1拉高,进入WiFi图传模式
    TR_CS_H;         // CS默认高电平(未选中)
    
    delay_us(100);   // 稳定延时
}

/**
 * @brief  等待WiFi模块准备好接收数据(IO2变高)
 * @param  wait_us: 最大等待时间(微秒)
 * @note   IO2=1表示模块准备好接收数据
 */
void TR_wait_startSign(uint16_t wait_us)
{
    uint32_t time = 0;

    while(1)
    {
        if(TR_IO2 == GPIO_PIN_SET)
        {
            break;  // 检测到开始信号
        }
        else
        {
            delay_us(50);
            time += 50;
            if(time > wait_us)
            {
                return;  // 超时退出
            }
        }
    }
}

/**
 * @brief  等待WiFi模块数据接收完成(IO2变低)
 * @param  wait_us: 最大等待时间(微秒)
 * @note   IO2=0表示模块接收完成
 */
void TR_wait_endSign(uint16_t wait_us)
{
    uint32_t time = 0;

    while(1)
    {
        if(TR_IO2 == GPIO_PIN_RESET)
        {
            break;  // 检测到结束信号
        }
        else
        {
            delay_us(50);
            time += 50;
            if(time > wait_us)
            {
                return;  // 超时退出
            }
        }
    }
}

/**
 * @brief  发送固定4000字节数据
 * @param  dat: 数据缓冲区指针(需要4000字节)
 * @note   分125次发送,每次32字节
 *         每次发送后延时53us,确保WiFi模块接收稳定
 */
void IR_Write_byte_4000(unsigned char *dat)
{
    unsigned char buff[32];

    TR_wait_startSign(200);  // 等待WiFi模块准备好
    TR_CS_L;                 // 片选使能

    for(int fre = 0; fre < 125; fre++)
    {
        memcpy(buff, &dat[32 * fre], 32);
        HAL_SPI_Transmit(&TR_SPI, buff, 32, 100);  // 发送32字节
        delay_us(53);                               // 数据间隔延时
    }
    
    delay_us(50);
    TR_CS_H;                 // 片选禁止
    TR_wait_endSign(200);    // 等待WiFi模块接收完成
}

/**
 * @brief  发送指定长度数据(小于4000字节)
 * @param  dat: 数据缓冲区指针
 * @param  len: 数据长度
 * @note   数据按32字节分包发送,不足32字节的部分补0
 */
void IR_Wirte_byte(unsigned char *dat, uint16_t len)
{
    unsigned short i;
    unsigned short fre = len / 32;   // 完整包数量
    unsigned short rem = len % 32;   // 剩余字节数
    unsigned char buff[32];

    TR_wait_startSign(200);
    TR_CS_L;

    // 发送完整的32字节包
    for(i = 0; i < fre; i++)
    {
        memcpy(buff, &dat[32 * i], 32);
        HAL_SPI_Transmit(&TR_SPI, buff, 32, 100);
        delay_us(53);
    }

    // 发送剩余不足32字节的数据
    if(rem != 0)
    {
        memset(buff, 0x00, 32);  // 清零缓冲区
        memcpy(buff, &dat[len - rem], rem);
        HAL_SPI_Transmit(&TR_SPI, buff, rem, 100);
    }

    delay_us(50);
    TR_CS_H;
    TR_wait_endSign(200);
}

/**
 * @brief  发送灰度图像
 * @param  high: 图像高度
 * @param  wide: 图像宽度
 * @param  dat: 图像数据指针(灰度值)
 * @note   图像格式: [文件头4字节] + [图像数据] + [文件尾4字节]
 *         数据量较大时分批发送(每批4000字节)
 */
void TR_Write_Image(unsigned char high, unsigned char wide, unsigned char *dat)
{
    unsigned short i;
    unsigned short temp = high * wide + 8;  // 总字节数
    unsigned char buff_T[4000];
    unsigned short frequency = temp / 4000;  // 需要发送的4000字节包数量
    unsigned short remainder = temp % 4000;  // 剩余字节数
    unsigned char img[TR_IMG_H * TR_IMG_W + 8];  // 图像缓冲区

    // 组合数据包: 文件头 + 图像数据 + 文件尾
    memcpy(&img[0], FH, 4);                  // 拷贝文件头
    memcpy(&img[4], dat, high * wide);       // 拷贝图像数据
    memcpy(&img[4 + high * wide], FE, 4);    // 拷贝文件尾

    // 分批发送4000字节数据包
    for(i = 0; i < frequency; i++)
    {
        memcpy(buff_T, &img[4000 * i], 4000);
        IR_Write_byte_4000(buff_T);
    }

    // 发送剩余数据
    if(remainder > 0)
    {
        memcpy(buff_T, &img[frequency * 4000], remainder);
        IR_Wirte_byte(buff_T, remainder);
    }
}

/**
 * @brief  发送二值化图像(像素压缩)
 * @param  height: 图像高度
 * @param  width: 图像宽度
 * @param  Pixle: 二值化图像数据指针(0或非0)
 * @note   8个像素压缩成1个字节,大幅减少传输数据量
 *         非0像素视为白色(1),0像素视为黑色(0)
 *         压缩格式: 按位打包,MSB优先
 */
void TR_Write_Image_Pixle(unsigned char height, unsigned char width, unsigned char *Pixle)
{
    uint8_t buff_T[32];
    unsigned long i;
    unsigned int pixel_total_bits = height * width;         // 总像素数
    unsigned int pixel_total_bytes = pixel_total_bits / 8;  // 压缩后字节数
    unsigned int total_bytes = pixel_total_bytes + 8;       // 总数据量
    unsigned int frequency = total_bytes / 32;              // 完整包数量
    unsigned char remainder = total_bytes % 32;             // 剩余字节数

    uint8_t img[TR_IMG_H * TR_IMG_W / 8 + 8];  // 压缩图像缓冲区

    memset(img, 0, sizeof(img));

    // 填充文件头
    memcpy(&img[0], FH, 4);

    // 压缩像素数据: 8个像素压缩成1个字节
    // Pixle已经是一维化的数组,直接按索引访问即可
    for(int idx = 0; idx < pixel_total_bits; idx++)
    {
        if(Pixle[idx] > 0)  // 非0视为白色
        {
            int byteIndex = idx / 8;
            int bitOffset = idx % 8;
            img[4 + byteIndex] |= (1 << (7 - bitOffset));  // MSB优先
        }
    }

    // 填充文件尾
    memcpy(&img[4 + pixel_total_bytes], FE, 4);

    // 发送压缩后的图像数据
    TR_wait_startSign(500);
    TR_CS_L;

    for(i = 0; i < frequency; i++)
    {
        memcpy(buff_T, &img[32 * i], 32);
        HAL_SPI_Transmit(&TR_SPI, buff_T, 32, 100);
        delay_us(53);
    }

    if(remainder != 0)
    {
        memset(buff_T, 0x00, 32);
        memcpy(buff_T, &img[total_bytes - remainder], remainder);
        HAL_SPI_Transmit(&TR_SPI, buff_T, remainder, 100);
        delay_us(53);
    }

    TR_CS_H;
    TR_wait_endSign(500);
}

/**
 ******************************************************************************
 * @brief  日志传输功能 - 适配UDP上位机
 ******************************************************************************
 */

/* 日志数据缓冲区 */
uint8_t g_log_buffer[LOG_BUFFER_SIZE];  // 全局日志缓冲区
uint16_t g_log_length = 0;               // 当前日志数据长度

/* 日志帧头尾标识 (与UDP上位机协议一致) */
static unsigned char LOG_FH[2] = {0xBB, 0x66};  // 日志帧头
static unsigned char LOG_FE[2] = {0x0D, 0x0A};  // 日志帧尾

/**
 * @brief  清空日志缓冲区
 * @note   在开始新的日志数据前调用
 */
void TR_Log_Clear(void)
{
    memset(g_log_buffer, 0, LOG_BUFFER_SIZE);
    g_log_length = 0;
}

/**
 * @brief  向日志缓冲区添加一个字节
 * @param  data: 要添加的字节数据
 * @retval 0-成功, -1-缓冲区已满
 */
int8_t TR_Log_AddByte(uint8_t data)
{
    if(g_log_length >= LOG_BUFFER_SIZE)
    {
        return -1;  // 缓冲区已满
    }
    g_log_buffer[g_log_length++] = data;
    return 0;
}

/**
 * @brief  向日志缓冲区添加多个字节
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 实际添加的字节数
 */
uint16_t TR_Log_AddBytes(uint8_t *data, uint16_t len)
{
    uint16_t remain = LOG_BUFFER_SIZE - g_log_length;
    uint16_t to_add = (len > remain) ? remain : len;
    
    memcpy(&g_log_buffer[g_log_length], data, to_add);
    g_log_length += to_add;
    
    return to_add;
}

/**
 * @brief  向日志缓冲区添加字符串
 * @param  str: 字符串指针(以'\0'结尾)
 * @retval 实际添加的字符数
 */
uint16_t TR_Log_AddString(const char *str)
{
    uint16_t len = strlen(str);
    return TR_Log_AddBytes((uint8_t*)str, len);
}

/**
 * @brief  向日志缓冲区添加uint8_t数值
 * @param  value: 数值
 * @retval 0-成功, -1-失败
 */
int8_t TR_Log_AddUint8(uint8_t value)
{
    return TR_Log_AddByte(value);
}

/**
 * @brief  向日志缓冲区添加uint16_t数值(小端序)
 * @param  value: 数值
 * @retval 0-成功, -1-失败
 */
int8_t TR_Log_AddUint16(uint16_t value)
{
    if(g_log_length + 2 > LOG_BUFFER_SIZE)
        return -1;
    
    g_log_buffer[g_log_length++] = (uint8_t)(value & 0xFF);        // 低字节
    g_log_buffer[g_log_length++] = (uint8_t)((value >> 8) & 0xFF); // 高字节
    return 0;
}

/**
 * @brief  向日志缓冲区添加uint32_t数值(小端序)
 * @param  value: 数值
 * @retval 0-成功, -1-失败
 */
int8_t TR_Log_AddUint32(uint32_t value)
{
    if(g_log_length + 4 > LOG_BUFFER_SIZE)
        return -1;
    
    g_log_buffer[g_log_length++] = (uint8_t)(value & 0xFF);
    g_log_buffer[g_log_length++] = (uint8_t)((value >> 8) & 0xFF);
    g_log_buffer[g_log_length++] = (uint8_t)((value >> 16) & 0xFF);
    g_log_buffer[g_log_length++] = (uint8_t)((value >> 24) & 0xFF);
    return 0;
}

/**
 * @brief  向日志缓冲区添加int8_t数值
 * @param  value: 数值
 * @retval 0-成功, -1-失败
 */
int8_t TR_Log_AddInt8(int8_t value)
{
    return TR_Log_AddByte((uint8_t)value);
}

/**
 * @brief  向日志缓冲区添加int16_t数值(小端序)
 * @param  value: 数值
 * @retval 0-成功, -1-失败
 */
int8_t TR_Log_AddInt16(int16_t value)
{
    return TR_Log_AddUint16((uint16_t)value);
}

/**
 * @brief  向日志缓冲区添加int32_t数值(小端序)
 * @param  value: 数值
 * @retval 0-成功, -1-失败
 */
int8_t TR_Log_AddInt32(int32_t value)
{
    return TR_Log_AddUint32((uint32_t)value);
}

/**
 * @brief  向日志缓冲区添加float数值(小端序)
 * @param  value: 浮点数
 * @retval 0-成功, -1-失败
 */
int8_t TR_Log_AddFloat(float value)
{
    if(g_log_length + 4 > LOG_BUFFER_SIZE)
        return -1;
    
    uint8_t *p = (uint8_t*)&value;
    for(int i = 0; i < 4; i++)
    {
        g_log_buffer[g_log_length++] = p[i];  // 小端序
    }
    return 0;
}

/**
 * @brief  向日志缓冲区添加float数组(小端序)
 * @param  array: float数组指针
 * @param  count: 数组元素个数
 * @retval 实际添加的float个数
 */
uint16_t TR_Log_AddFloatArray(float *array, uint16_t count)
{
    uint16_t added = 0;
    
    for(uint16_t i = 0; i < count; i++)
    {
        if(TR_Log_AddFloat(array[i]) == 0)
        {
            added++;
        }
        else
        {
            break;  // 缓冲区满,停止添加
        }
    }
    
    return added;
}

/**
 * @brief  获取日志缓冲区指针(只读)
 * @retval 日志缓冲区指针
 */
const uint8_t* TR_Log_GetBuffer(void)
{
    return g_log_buffer;
}

/**
 * @brief  获取当前日志数据长度
 * @retval 日志数据长度
 */
uint16_t TR_Log_GetLength(void)
{
    return g_log_length;
}

/**
 * @brief  发送日志数据(纯文本格式 - 推荐)
 * @note   适配UDP上位机日志帧格式(纯文本模式)
 *         数据包格式: [帧头BB66] + [日志内容] + [帧尾0D0A]
 *         发送前需要先调用 TR_Log_AddXXX() 函数填充数据
 */
void TR_Send_Log(void)
{
    if(g_log_length == 0)
        return;  // 没有数据,不发送
    
    uint8_t send_buff[32];
    uint16_t total_bytes = 2 + g_log_length + 2;  // 帧头(2) + 数据 + 帧尾(2)
    uint16_t frequency = total_bytes / 32;
    uint8_t remainder = total_bytes % 32;
    
    // 临时缓冲区(包含帧头帧尾)
    uint8_t packet[LOG_BUFFER_SIZE + 4];
    memcpy(&packet[0], LOG_FH, 2);                 // 拷贝帧头
    memcpy(&packet[2], g_log_buffer, g_log_length); // 拷贝数据
    memcpy(&packet[2 + g_log_length], LOG_FE, 2);  // 拷贝帧尾
    
    // 分包发送
    TR_wait_startSign(200);
    TR_CS_L;
    
    for(uint16_t i = 0; i < frequency; i++)
    {
        memcpy(send_buff, &packet[32 * i], 32);
        HAL_SPI_Transmit(&TR_SPI, send_buff, 32, 100);
        delay_us(53);
    }
    
    if(remainder != 0)
    {
        memset(send_buff, 0x00, 32);
        memcpy(send_buff, &packet[total_bytes - remainder], remainder);
        HAL_SPI_Transmit(&TR_SPI, send_buff, remainder, 100);
        delay_us(53);
    }
    
    TR_CS_H;
    TR_wait_endSign(200);
    
    // 发送完成后清空缓冲区
    TR_Log_Clear();
}

/**
 * @brief  发送日志数据(标准格式 - 带长度字段)
 * @note   适配UDP上位机日志帧格式(标准格式)
 *         数据包格式: [帧头BB66] + [0x02] + [LEN] + [日志内容] + [帧尾0D0A]
 *         LEN: 1字节,表示日志内容长度(不含类型/长度字段)
 */
void TR_Send_Log_Standard(void)
{
    if(g_log_length == 0)
        return;
    
    if(g_log_length > 255)
        return;  // 标准格式长度不能超过255
    
    uint8_t send_buff[32];
    uint16_t total_bytes = 2 + 1 + 1 + g_log_length + 2;  // 帧头+类型+长度+数据+帧尾
    uint16_t frequency = total_bytes / 32;
    uint8_t remainder = total_bytes % 32;
    
    // 临时缓冲区
    uint8_t packet[LOG_BUFFER_SIZE + 6];
    uint16_t idx = 0;
    
    memcpy(&packet[idx], LOG_FH, 2);        // 帧头
    idx += 2;
    packet[idx++] = 0x02;                   // 类型字段(固定0x02)
    packet[idx++] = (uint8_t)g_log_length; // 长度字段
    memcpy(&packet[idx], g_log_buffer, g_log_length);  // 数据
    idx += g_log_length;
    memcpy(&packet[idx], LOG_FE, 2);       // 帧尾
    
    // 分包发送
    TR_wait_startSign(200);
    TR_CS_L;
    
    for(uint16_t i = 0; i < frequency; i++)
    {
        memcpy(send_buff, &packet[32 * i], 32);
        HAL_SPI_Transmit(&TR_SPI, send_buff, 32, 100);
        delay_us(53);
    }
    
    if(remainder != 0)
    {
        memset(send_buff, 0x00, 32);
        memcpy(send_buff, &packet[total_bytes - remainder], remainder);
        HAL_SPI_Transmit(&TR_SPI, send_buff, remainder, 100);
        delay_us(53);
    }
    
    TR_CS_H;
    TR_wait_endSign(200);
    
    TR_Log_Clear();
}

/**
 * @brief  快速发送单字节日志
 * @param  data: 字节数据
 */
void TR_Send_Log_Byte(uint8_t data)
{
    TR_Log_Clear();
    TR_Log_AddByte(data);
    TR_Send_Log();
}

/**
 * @brief  快速发送字符串日志
 * @param  str: 字符串(以'\0'结尾)
 */
void TR_Send_Log_String(const char *str)
{
    TR_Log_Clear();
    TR_Log_AddString(str);
    TR_Send_Log();
}

/**
 * ---------------------------------------------------------------------------
 * 接收相关函数 (最小侵入实现，使用HAL SPI)
 * ---------------------------------------------------------------------------
 */

/**
 * @brief  从模块读取固定4000字节数据
 * @param  dat: 目标缓冲区，需至少4000字节
 * @note   与发送对称，实现分125次、每次32字节接收
 */
void IR_Read_byte_4000(unsigned char *dat)
{
    unsigned char buff[32];

    TR_wait_startSign(200);  // 等待模块开始
    TR_CS_L;

    for(int fre = 0; fre < 125; fre++)
    {
        memset(buff, 0, sizeof(buff));
        HAL_SPI_Receive(&TR_SPI, buff, 32, 100);
        memcpy(&dat[32 * fre], buff, 32);
        delay_us(53);
    }

    delay_us(50);
    TR_CS_H;
    TR_wait_endSign(200);
}


/**
 * @brief  从模块读取指定长度的数据(小于等于4000)
 * @param  dat: 目标缓冲区
 * @param  len: 要读取的字节数
 * @note   按32字节分包读取，不足32字节的部分会用0填充接收缓冲
 */
void IR_Read_byte(unsigned char *dat, unsigned short len)
{
    unsigned short i;
    unsigned short fre = len / 32;   // 完整包数量
    unsigned short rem = len % 32;   // 剩余字节数
    unsigned char buff[32];

    TR_wait_startSign(200);
    TR_CS_L;

    for(i = 0; i < fre; i++)
    {
        memset(buff, 0, sizeof(buff));
        HAL_SPI_Receive(&TR_SPI, buff, 32, 100);
        memcpy(&dat[32 * i], buff, 32);
        delay_us(53);
    }

    if(rem != 0)
    {
        memset(buff, 0x00, 32);
        HAL_SPI_Receive(&TR_SPI, buff, rem, 100);
        memcpy(&dat[len - rem], buff, rem);
    }

    delay_us(50);
    TR_CS_H;
    TR_wait_endSign(200);
}


/**
 * @brief  接收一帧数据，帧以FH开头，以FE结尾
 * @param  out_buf: 输出缓冲区
 * @param  max_len: 缓冲区最大长度
 * @param  timeout_ms: 超时时间（毫秒）
 * @return 实际接收的字节数，超时或出错返回0
 *
 * @note  该实现以最小侵入为原则：重复从模块读取32字节块，
 *       按字节检查帧头(4字节)与帧尾(4字节)。该函数可能会阻塞，
 *       直到接收到完整帧或超时。
 */
unsigned short TR_Receive_Packet(unsigned char *out_buf, unsigned short max_len, unsigned long timeout_ms)
{
    unsigned long start = HAL_GetTick();
    unsigned int total = 0;
    unsigned char chunk[32];
    unsigned char window[8]; // 用于检查头尾
    unsigned int i;
    int found_header = 0;
    unsigned int header_pos = 0;

    if(out_buf == NULL || max_len == 0)
        return 0;

    memset(window, 0, sizeof(window));

    while((HAL_GetTick() - start) < timeout_ms)
    {
        // 每次尝试读取32字节（或直到超时）
        memset(chunk, 0, sizeof(chunk));
        // 为避免长期阻塞，先判断模块是否有开始信号
        if(TR_IO2 != GPIO_PIN_SET)
        {
            // 没有数据准备，短延时后重试
            delay_us(500);
            continue;
        }

        IR_Read_byte(chunk, 32);

        for(i = 0; i < 32; i++)
        {
            // 滑动窗口维护最近8字节用于头尾检测
            for(int k = 0; k < 7; k++)
                window[k] = window[k+1];
            window[7] = chunk[i];

            if(!found_header)
            {
                // 检查是否出现帧头 FH[4]
                if(window[4] == FH[0] && window[5] == FH[1] && window[6] == FH[2] && window[7] == FH[3])
                {
                    found_header = 1;
                    // 将帧头也写入输出
                    if(total + 4 <= max_len)
                    {
                        out_buf[total++] = FH[0];
                        out_buf[total++] = FH[1];
                        out_buf[total++] = FH[2];
                        out_buf[total++] = FH[3];
                    }
                    else
                    {
                        return 0; // 空间不足
                    }
                }
            }
            else
            {
                // 已找到头，继续收集数据
                if(total < max_len)
                {
                    out_buf[total++] = chunk[i];
                }
                else
                {
                    return 0; // 空间溢出
                }

                // 检查是否出现帧尾 FE[4]
                if(total >= 4)
                {
                    unsigned int end_idx = total - 4;
                    if(out_buf[end_idx] == FE[0] && out_buf[end_idx+1] == FE[1] && out_buf[end_idx+2] == FE[2] && out_buf[end_idx+3] == FE[3])
                    {
                        // 完整帧接收完成
                        return (unsigned short)total;
                    }
                }
            }
        }

        // 若未找到头，继续循环等待
    }

    // 超时未收到完整帧
    return 0;
}


/**
 * ---------------------------------------------------------------------------
 * 接收数据解析函数 (从接收缓冲区读取不同数据类型)
 * ---------------------------------------------------------------------------
 */

/**
 * @brief  从接收缓冲区读取uint8_t
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 字节索引(从0开始)
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Uint8(const unsigned char *buf, unsigned short buf_len, unsigned short index, uint8_t *out_value)
{
    if(buf == NULL || out_value == NULL)
        return -1;
    
    if(index >= buf_len)
        return -1;
    
    *out_value = buf[index];
    return 0;
}

/**
 * @brief  从接收缓冲区读取uint16_t(小端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Uint16(const unsigned char *buf, unsigned short buf_len, unsigned short index, uint16_t *out_value)
{
    if(buf == NULL || out_value == NULL)
        return -1;
    
    if(index + 1 >= buf_len)
        return -1;
    
    *out_value = (uint16_t)buf[index] | ((uint16_t)buf[index + 1] << 8);
    return 0;
}

/**
 * @brief  从接收缓冲区读取uint32_t(小端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Uint32(const unsigned char *buf, unsigned short buf_len, unsigned short index, uint32_t *out_value)
{
    if(buf == NULL || out_value == NULL)
        return -1;
    
    if(index + 3 >= buf_len)
        return -1;
    
    *out_value = (uint32_t)buf[index] 
               | ((uint32_t)buf[index + 1] << 8)
               | ((uint32_t)buf[index + 2] << 16)
               | ((uint32_t)buf[index + 3] << 24);
    return 0;
}

/**
 * @brief  从接收缓冲区读取int8_t
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Int8(const unsigned char *buf, unsigned short buf_len, unsigned short index, int8_t *out_value)
{
    if(buf == NULL || out_value == NULL)
        return -1;
    
    if(index >= buf_len)
        return -1;
    
    *out_value = (int8_t)buf[index];
    return 0;
}

/**
 * @brief  从接收缓冲区读取int16_t(小端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Int16(const unsigned char *buf, unsigned short buf_len, unsigned short index, int16_t *out_value)
{
    uint16_t temp;
    int8_t ret = TR_Read_Uint16(buf, buf_len, index, &temp);
    if(ret == 0)
        *out_value = (int16_t)temp;
    return ret;
}

/**
 * @brief  从接收缓冲区读取int32_t(小端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Int32(const unsigned char *buf, unsigned short buf_len, unsigned short index, int32_t *out_value)
{
    uint32_t temp;
    int8_t ret = TR_Read_Uint32(buf, buf_len, index, &temp);
    if(ret == 0)
        *out_value = (int32_t)temp;
    return ret;
}

/**
 * @brief  从接收缓冲区读取float(小端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Float(const unsigned char *buf, unsigned short buf_len, unsigned short index, float *out_value)
{
    if(buf == NULL || out_value == NULL)
        return -1;
    
    if(index + 3 >= buf_len)
        return -1;
    
    uint8_t *p = (uint8_t*)out_value;
    for(int i = 0; i < 4; i++)
    {
        p[i] = buf[index + i];  // 小端序
    }
    return 0;
}

/**
 * @brief  从接收缓冲区读取uint16_t(大端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Uint16_BE(const unsigned char *buf, unsigned short buf_len, unsigned short index, uint16_t *out_value)
{
    if(buf == NULL || out_value == NULL)
        return -1;
    
    if(index + 1 >= buf_len)
        return -1;
    
    *out_value = ((uint16_t)buf[index] << 8) | (uint16_t)buf[index + 1];
    return 0;
}

/**
 * @brief  从接收缓冲区读取uint32_t(大端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Uint32_BE(const unsigned char *buf, unsigned short buf_len, unsigned short index, uint32_t *out_value)
{
    if(buf == NULL || out_value == NULL)
        return -1;
    
    if(index + 3 >= buf_len)
        return -1;
    
    *out_value = ((uint32_t)buf[index] << 24)
               | ((uint32_t)buf[index + 1] << 16)
               | ((uint32_t)buf[index + 2] << 8)
               | (uint32_t)buf[index + 3];
    return 0;
}

/**
 * @brief  从接收缓冲区读取int16_t(大端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Int16_BE(const unsigned char *buf, unsigned short buf_len, unsigned short index, int16_t *out_value)
{
    uint16_t temp;
    int8_t ret = TR_Read_Uint16_BE(buf, buf_len, index, &temp);
    if(ret == 0)
        *out_value = (int16_t)temp;
    return ret;
}

/**
 * @brief  从接收缓冲区读取int32_t(大端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Int32_BE(const unsigned char *buf, unsigned short buf_len, unsigned short index, int32_t *out_value)
{
    uint32_t temp;
    int8_t ret = TR_Read_Uint32_BE(buf, buf_len, index, &temp);
    if(ret == 0)
        *out_value = (int32_t)temp;
    return ret;
}

/**
 * @brief  从接收缓冲区读取float(大端序)
 * @param  buf: 接收缓冲区指针
 * @param  buf_len: 缓冲区长度
 * @param  index: 起始字节索引
 * @param  out_value: 输出值指针
 * @retval 0-成功, -1-索引越界
 */
int8_t TR_Read_Float_BE(const unsigned char *buf, unsigned short buf_len, unsigned short index, float *out_value)
{
    if(buf == NULL || out_value == NULL)
        return -1;
    
    if(index + 3 >= buf_len)
        return -1;
    
    uint8_t *p = (uint8_t*)out_value;
    for(int i = 0; i < 4; i++)
    {
        p[3 - i] = buf[index + i];  // 大端序转小端
    }
    return 0;
}
