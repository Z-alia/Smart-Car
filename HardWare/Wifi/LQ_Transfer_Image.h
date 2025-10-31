#ifndef _LQ_TRANSFER_IMAGE_H_
#define _LQ_TRANSFER_IMAGE_H_

#include "main.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>

/* WiFi模块引脚定义 - 根据您的硬件连接修改 */
#define TR_CS_PORT        WIFI_CS_GPIO_Port      // WiFi CS引脚 - PB12
#define TR_CS_PIN         WIFI_CS_Pin

#define TR_IO1_PORT       wifi_io1_GPIO_Port     // WiFi IO1引脚 - PD11 (模式配置)
#define TR_IO1_PIN        wifi_io1_Pin

#define TR_IO2_PORT       wifi_io2_GPIO_Port     // WiFi IO2引脚 - PB6 (握手信号)
#define TR_IO2_PIN        wifi_io2_Pin

/* SPI接口定义 - 使用SPI3 */
#define TR_SPI            hspi2                  // 使用SPI2

/* GPIO操作宏定义 */
#define TR_CS_H           HAL_GPIO_WritePin(TR_CS_PORT, TR_CS_PIN, GPIO_PIN_SET)
#define TR_CS_L           HAL_GPIO_WritePin(TR_CS_PORT, TR_CS_PIN, GPIO_PIN_RESET)

#define TR_IO1_H          HAL_GPIO_WritePin(TR_IO1_PORT, TR_IO1_PIN, GPIO_PIN_SET)
#define TR_IO1_L          HAL_GPIO_WritePin(TR_IO1_PORT, TR_IO1_PIN, GPIO_PIN_RESET)

#define TR_IO2            HAL_GPIO_ReadPin(TR_IO2_PORT, TR_IO2_PIN)

/* 图像尺寸定义 */
#define TR_IMG_W          188
#define TR_IMG_H          120

/* 数据包头尾标识 */
extern unsigned char FH[4];
extern unsigned char FE[4];

/* 函数声明 - 图像传输 */
void TR_driver_init(void);
void TR_wait_startSign(uint16_t wait_us);
void TR_wait_endSign(uint16_t wait_us);
void IR_Write_byte_4000(unsigned char *dat);
void IR_Wirte_byte(unsigned char *dat, uint16_t len);
void TR_Write_Image(unsigned char high, unsigned char wide, unsigned char *dat);
void TR_Write_Image_Pixle(unsigned char height, unsigned char width, unsigned char *Pixle);

/* 日志缓冲区全局变量声明 (可在外部直接访问和赋值) */
#define LOG_BUFFER_SIZE 32
extern uint8_t g_log_buffer[LOG_BUFFER_SIZE];  // 日志数据缓冲区
extern uint16_t g_log_length;                   // 日志数据当前长度

/* 函数声明 - 日志传输(适配UDP上位机) */
void TR_Log_Clear(void);                                // 清空日志缓冲区
int8_t TR_Log_AddByte(uint8_t data);                    // 添加单字节
uint16_t TR_Log_AddBytes(uint8_t *data, uint16_t len); // 添加字节数组
uint16_t TR_Log_AddString(const char *str);            // 添加字符串
int8_t TR_Log_AddUint8(uint8_t value);                 // 添加uint8_t
int8_t TR_Log_AddUint16(uint16_t value);               // 添加uint16_t(小端)
int8_t TR_Log_AddUint32(uint32_t value);               // 添加uint32_t(小端)
int8_t TR_Log_AddInt8(int8_t value);                   // 添加int8_t
int8_t TR_Log_AddInt16(int16_t value);                 // 添加int16_t(小端)
int8_t TR_Log_AddInt32(int32_t value);                 // 添加int32_t(小端)
int8_t TR_Log_AddFloat(float value);                   // 添加float(小端)
uint16_t TR_Log_AddFloatArray(float *array, uint16_t count);  // 添加float数组
const uint8_t* TR_Log_GetBuffer(void);                 // 获取日志缓冲区指针
uint16_t TR_Log_GetLength(void);                       // 获取日志长度
void TR_Send_Log(void);                                 // 发送日志(纯文本格式-推荐)
void TR_Send_Log_Standard(void);                        // 发送日志(标准格式-带长度)
void TR_Send_Log_Byte(uint8_t data);                   // 快速发送单字节
void TR_Send_Log_String(const char *str);              // 快速发送字符串

/* 延时函数声明 - 需要在tim.c中实现或使用HAL_Delay */
void delay_us(uint32_t us);

#endif /* _LQ_TRANSFER_IMAGE_H_ */
