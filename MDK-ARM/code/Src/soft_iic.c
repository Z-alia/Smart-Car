#include "soft_iic.h"

#define SOFT_IIC_SDA_IO_SWITCH          (1)            // 是否需要 SDA 进行 I/O 切换 0-不需要 1-需要

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 延时
// 参数说明     delay           延时次数
// 返回参数     void
// 使用示例     soft_iic_delay(1);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
//static void soft_iic_delay (volatile uint32_t delay)
//{
//    volatile uint32 count = delay;
//    while(count --);
//}
// 软件延时
#define soft_iic_delay(x)  for(volatile uint32_t i = x; i--; )

// === GPIO 操作宏 ===
#define SDA_HIGH(obj) HAL_GPIO_WritePin((obj)->sda_port, (obj)->sda_pin, GPIO_PIN_SET)
#define SDA_LOW(obj)  HAL_GPIO_WritePin((obj)->sda_port, (obj)->sda_pin, GPIO_PIN_RESET)
#define SCL_HIGH(obj) HAL_GPIO_WritePin((obj)->scl_port, (obj)->scl_pin, GPIO_PIN_SET)
#define SCL_LOW(obj)  HAL_GPIO_WritePin((obj)->scl_port, (obj)->scl_pin, GPIO_PIN_RESET)

#define SDA_READ(obj) HAL_GPIO_ReadPin((obj)->sda_port, (obj)->sda_pin)

// === SDA 设置为输入/输出 ===
static void SDA_IN(soft_iic_info_struct *obj)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = obj->sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(obj->sda_port, &GPIO_InitStruct);
}

static void SDA_OUT(soft_iic_info_struct *obj)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = obj->sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(obj->sda_port, &GPIO_InitStruct);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC START 信号
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 返回参数     void
// 使用示例     soft_iic_start(soft_iic_obj);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void soft_iic_start (soft_iic_info_struct *soft_iic_obj)
{
	SDA_OUT(soft_iic_obj);
    SCL_HIGH(soft_iic_obj);// SCL 高电平
    SDA_HIGH(soft_iic_obj); // SDA 高电平
    soft_iic_delay(soft_iic_obj->delay);
    SDA_LOW(soft_iic_obj);// SDA 先拉低
    soft_iic_delay(soft_iic_obj->delay);
    SCL_LOW(soft_iic_obj); // SCL 再拉低
    soft_iic_delay(soft_iic_obj->delay);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC STOP 信号
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 返回参数     void
// 使用示例     soft_iic_stop(soft_iic_obj);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void soft_iic_stop (soft_iic_info_struct *soft_iic_obj)
{
	SDA_OUT(soft_iic_obj);
    SCL_LOW(soft_iic_obj);// SCL 低电平
    SDA_LOW(soft_iic_obj);// SDA 低电平
    soft_iic_delay(soft_iic_obj->delay);
    SCL_HIGH(soft_iic_obj);// SCL 先拉高
    soft_iic_delay(soft_iic_obj->delay);
    SDA_HIGH(soft_iic_obj);// SDA 再拉高
    soft_iic_delay(soft_iic_obj->delay);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 发送 ACK/NAKC 信号 内部调用
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     ack             ACK 电平
// 返回参数     void
// 使用示例     soft_iic_send_ack(soft_iic_obj, 1);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void soft_iic_send_ack (soft_iic_info_struct *soft_iic_obj, uint8_t ack)
{
	SCL_LOW(soft_iic_obj);// SCL 低电平
    SDA_OUT(soft_iic_obj);
    if(ack) 
	{
		SDA_HIGH(soft_iic_obj); // SDA 拉高
	}
    else    
	{
		SDA_LOW(soft_iic_obj);// SDA 拉低
	}
    soft_iic_delay(soft_iic_obj->delay);
    SCL_HIGH(soft_iic_obj);// SCL 拉高
    soft_iic_delay(soft_iic_obj->delay);
    SCL_LOW(soft_iic_obj);// SCL 拉低
    SDA_HIGH(soft_iic_obj); // 释放 SDA
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 获取 ACK/NAKC 信号
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 返回参数     uint8           ACK 状态
// 使用示例     soft_iic_wait_ack(soft_iic_obj);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t soft_iic_wait_ack (soft_iic_info_struct *soft_iic_obj)
{
	uint8_t ack;
    SCL_LOW(soft_iic_obj);// SCL 低电平
    SDA_IN(soft_iic_obj);     // 释放 SDA
    SDA_HIGH(soft_iic_obj);
    soft_iic_delay(soft_iic_obj->delay);
    SCL_HIGH(soft_iic_obj);// SCL 高电平
    soft_iic_delay(soft_iic_obj->delay);
    ack = SDA_READ(soft_iic_obj);
    SCL_LOW(soft_iic_obj);// SCL 低电平
    SDA_OUT(soft_iic_obj);
    return ack ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 发送 8bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     data            数据
// 返回参数     uint8           ACK 状态
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t soft_iic_send_data (soft_iic_info_struct *soft_iic_obj, const uint8_t data)
{
	SDA_OUT(soft_iic_obj);
    for(uint8_t mask = 0x80; mask; mask >>= 1)
    {
        if(data & mask) SDA_HIGH(soft_iic_obj);
        else            SDA_LOW(soft_iic_obj);
        soft_iic_delay(soft_iic_obj->delay / 2);
        SCL_HIGH(soft_iic_obj);// SCL 拉高
        soft_iic_delay(soft_iic_obj->delay);
        SCL_LOW(soft_iic_obj);// SCL 拉低
        soft_iic_delay(soft_iic_obj->delay / 2);
    }
    return soft_iic_wait_ack(soft_iic_obj) ? 0 : 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 读取 8bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     ack             ACK 或 NACK
// 返回参数     uint8           数据
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t soft_iic_read_data (soft_iic_info_struct *soft_iic_obj, uint8_t ack)
{
	uint8_t data = 0;
    SDA_IN(soft_iic_obj);
    for(uint8_t i = 0; i < 8; i++)
    {
        SCL_LOW(soft_iic_obj);// SCL 低电平
        soft_iic_delay(soft_iic_obj->delay);
        SCL_HIGH(soft_iic_obj);
        soft_iic_delay(soft_iic_obj->delay);
        data = (data << 1) | SDA_READ(soft_iic_obj);
    }
    SCL_LOW(soft_iic_obj);// SCL 拉低
    SDA_OUT(soft_iic_obj);
    soft_iic_send_ack(soft_iic_obj, ack);
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口写 8bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     data            要写入的数据
// 返回参数     void            
// 使用示例     soft_iic_write_8bit_register(soft_iic_obj, 0x01);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_write_8bit (soft_iic_info_struct *soft_iic_obj, const uint8_t data)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, data);
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口写 8bit 数组
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照soft_iic.h 里的格式看看
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             缓冲区长度
// 返回参数     void            
// 使用示例     soft_iic_write_8bit_array(soft_iic_obj, data, 6);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_write_8bit_array (soft_iic_info_struct *soft_iic_obj, const uint8_t *data, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    while(len --)
    {
        soft_iic_send_data(soft_iic_obj, *data ++);
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口器写 16bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 oft_iic.h 里的格式看看
// 参数说明     data            要写入的数据
// 返回参数     void            
// 使用示例     soft_iic_write_16bit(soft_iic_obj, 0x0101);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_write_16bit (soft_iic_info_struct *soft_iic_obj, const uint16_t data)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, (uint8_t)((data & 0xFF00) >> 8));
    soft_iic_send_data(soft_iic_obj, (uint8_t)(data & 0x00FF));
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口写 16bit 数组
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             缓冲区长度
// 返回参数     void            
// 使用示例     soft_iic_write_16bit_array(soft_iic_obj, data, 6);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_write_16bit_array (soft_iic_info_struct *soft_iic_obj, const uint16_t *data, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    while(len --)
    {
        soft_iic_send_data(soft_iic_obj, (uint8_t)((*data & 0xFF00) >> 8));
        soft_iic_send_data(soft_iic_obj, (uint8_t)(*data ++ & 0x00FF));
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口向传感器寄存器写 8bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 参数说明     data            要写入的数据
// 返回参数     void            
// 使用示例     soft_iic_write_8bit_register(soft_iic_obj, 0x01, 0x01);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_write_8bit_register (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, const uint8_t data)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, register_name);
    soft_iic_send_data(soft_iic_obj, data);
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口向传感器寄存器写 8bit 数组
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             缓冲区长度
// 返回参数     void            
// 使用示例     soft_iic_write_8bit_registers(soft_iic_obj, 0x01, data, 6);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_write_8bit_registers (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, const uint8_t *data, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, register_name);
    while(len --)
    {
        soft_iic_send_data(soft_iic_obj, *data ++);
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口向传感器寄存器写 16bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 参数说明     data            要写入的数据
// 返回参数     void            
// 使用示例     soft_iic_write_16bit_register(soft_iic_obj, 0x0101, 0x0101);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_write_16bit_register (soft_iic_info_struct *soft_iic_obj, const uint16_t register_name, const uint16_t data)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, (uint8_t)((register_name & 0xFF00) >> 8));
    soft_iic_send_data(soft_iic_obj, (uint8_t)(register_name & 0x00FF));
    soft_iic_send_data(soft_iic_obj, (uint8_t)((data & 0xFF00) >> 8));
    soft_iic_send_data(soft_iic_obj, (uint8_t)(data & 0x00FF));
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口向传感器寄存器写 16bit 数组
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             缓冲区长度
// 返回参数     void            
// 使用示例     soft_iic_write_16bit_registers(soft_iic_obj, 0x0101, data, 6);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_write_16bit_registers (soft_iic_info_struct *soft_iic_obj, const uint16_t register_name, const uint16_t *data, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, (uint8_t)((register_name & 0xFF00) >> 8));
    soft_iic_send_data(soft_iic_obj, (uint8_t)(register_name & 0x00FF));
    while(len --)
    {
        soft_iic_send_data(soft_iic_obj, (uint8_t)((*data & 0xFF00) >> 8));
        soft_iic_send_data(soft_iic_obj, (uint8_t)(*data ++ & 0x00FF));
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口读取 8bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 返回参数     uint8           返回读取的 8bit 数据
// 使用示例     soft_iic_read_8bit(soft_iic_obj);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8_t soft_iic_read_8bit (soft_iic_info_struct *soft_iic_obj)
{
    uint8_t temp = 0;
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    temp = soft_iic_read_data(soft_iic_obj, 1);
    soft_iic_stop(soft_iic_obj);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口从传感器寄存器读取 8bit 数组
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 参数说明     *data           要读取的数据的缓冲区指针
// 参数说明     len             要读取的数据长度
// 返回参数     void            
// 使用示例     soft_iic_read_8bit_array(soft_iic_obj, data, 8);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_read_8bit_array (soft_iic_info_struct *soft_iic_obj, uint8_t *data, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    while(len --)
    {
        *data ++ = soft_iic_read_data(soft_iic_obj, len == 0);
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口读取 16bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 返回参数     uint16          返回读取的 16bit 数据
// 使用示例     soft_iic_read_16bit(soft_iic_obj);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint16_t soft_iic_read_16bit (soft_iic_info_struct *soft_iic_obj)
{
    uint16_t temp = 0;
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    temp = soft_iic_read_data(soft_iic_obj, 0);
    temp = ((temp << 8)| soft_iic_read_data(soft_iic_obj, 1));
    soft_iic_stop(soft_iic_obj);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口读取 16bit 数组
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     *data           要读取的数据的缓冲区指针
// 参数说明     len             要读取的数据长度
// 返回参数     void            
// 使用示例     soft_iic_read_16bit_array(soft_iic_obj, data, 8);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_read_16bit_array (soft_iic_info_struct *soft_iic_obj, uint16_t *data, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    while(len --)
    {
        *data = soft_iic_read_data(soft_iic_obj, 0);
        *data = ((*data << 8)| soft_iic_read_data(soft_iic_obj, 0 == len));
        data ++;
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口从传感器寄存器读取 8bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 返回参数     uint8           返回读取的 8bit 数据
// 使用示例     soft_iic_read_8bit_register(soft_iic_obj, 0x01);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8_t soft_iic_read_8bit_register (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name)
{
    uint8_t temp = 0;
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, register_name);
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    temp = soft_iic_read_data(soft_iic_obj, 1);
    soft_iic_stop(soft_iic_obj);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口从传感器寄存器读取 8bit 数组
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 参数说明     *data           要读取的数据的缓冲区指针
// 参数说明     len             要读取的数据长度
// 返回参数     void            
// 使用示例     soft_iic_read_8bit_registers(soft_iic_obj, 0x01, data, 8);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_read_8bit_registers (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, uint8_t *data, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, register_name);
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    while(len --)
    {
        *data ++ = soft_iic_read_data(soft_iic_obj, len == 0);
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口从传感器寄存器读取 16bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 返回参数     uint16          返回读取的 16bit 数据
// 使用示例     soft_iic_read_16bit_register(soft_iic_obj, 0x0101);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint16_t soft_iic_read_16bit_register (soft_iic_info_struct *soft_iic_obj, const uint16_t register_name)
{
    uint16_t temp = 0;
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, (uint8_t)((register_name & 0xFF00) >> 8));
    soft_iic_send_data(soft_iic_obj, (uint8_t)(register_name & 0x00FF));
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    temp = soft_iic_read_data(soft_iic_obj, 0);
    temp = ((temp << 8)| soft_iic_read_data(soft_iic_obj, 1));
    soft_iic_stop(soft_iic_obj);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口从传感器寄存器读取 16bit 数组
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 参数说明     *data           要读取的数据的缓冲区指针
// 参数说明     len             要读取的数据长度
// 返回参数     void            
// 使用示例     soft_iic_read_16bit_registers(soft_iic_obj, 0x0101, data, 8);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_read_16bit_registers (soft_iic_info_struct *soft_iic_obj, const uint16_t register_name, uint16_t *data, uint32_t len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, (uint8_t)((register_name & 0xFF00) >> 8));
    soft_iic_send_data(soft_iic_obj, (uint8_t)(register_name & 0x00FF));
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    while(len --)
    {
        *data = soft_iic_read_data(soft_iic_obj, 0);
        *data = ((*data << 8)| soft_iic_read_data(soft_iic_obj, 0 == len));
        data ++;
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口传输 8bit 数组 先写后读取
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     *write_data     发送数据存放缓冲区
// 参数说明     write_len       发送缓冲区长度
// 参数说明     *read_data      读取数据存放缓冲区
// 参数说明     read_len        读取缓冲区长度
// 返回参数     void            
// 使用示例     iic_transfer_8bit_array(IIC_1, addr, data, 64, data, 64);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_transfer_8bit_array (soft_iic_info_struct *soft_iic_obj, const uint8_t *write_data, uint32_t write_len, uint8_t *read_data, uint32_t read_len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    while(write_len --)
    {
        soft_iic_send_data(soft_iic_obj, *write_data ++);
    }
    if(read_len)
    {
        soft_iic_start(soft_iic_obj);
        soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
        while(read_len --)
        {
            *read_data ++ = soft_iic_read_data(soft_iic_obj, 0 == read_len);
        }
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口传输 16bit 数组 先写后读取
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 zsoft_iic.h 里的格式看看
// 参数说明     *write_data     发送数据存放缓冲区
// 参数说明     write_len       发送缓冲区长度
// 参数说明     *read_data      读取数据存放缓冲区
// 参数说明     read_len        读取缓冲区长度
// 返回参数     void            
// 使用示例     iic_transfer_16bit_array(IIC_1, addr, data, 64, data, 64);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_transfer_16bit_array (soft_iic_info_struct *soft_iic_obj, const uint16_t *write_data, uint32_t write_len, uint16_t *read_data, uint32_t read_len)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    while(write_len--)
    {
        soft_iic_send_data(soft_iic_obj, (uint8_t)((*write_data & 0xFF00) >> 8));
        soft_iic_send_data(soft_iic_obj, (uint8_t)(*write_data ++ & 0x00FF));
    }
    if(read_len)
    {
        soft_iic_start(soft_iic_obj);
        soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
        while(read_len --)
        {
            *read_data = soft_iic_read_data(soft_iic_obj, 0);
            *read_data = ((*read_data << 8)| soft_iic_read_data(soft_iic_obj, 0 == read_len));
            read_data ++;
        }
    }
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口 SCCB 模式向传感器寄存器写 8bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 
// 参数说明     register_name   传感器的寄存器地址
// 参数说明     data            要写入的数据
// 返回参数     void            
// 使用示例     soft_iic_sccb_write_register(soft_iic_obj, 0x01, 0x01);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_sccb_write_register (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, uint8_t data)
{
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, register_name);
    soft_iic_send_data(soft_iic_obj, data);
    soft_iic_stop(soft_iic_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口 SCCB 模式从传感器寄存器读取 8bit 数据
// 参数说明     *soft_iic_obj   软件 IIC 指定信息 可以参照 soft_iic.h 里的格式看看
// 参数说明     register_name   传感器的寄存器地址
// 返回参数     uint8           返回读取的 8bit 数据
// 使用示例     soft_iic_sccb_read_register(soft_iic_obj, 0x01);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8_t soft_iic_sccb_read_register (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name)
{
    uint8_t temp = 0;
    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1);
    soft_iic_send_data(soft_iic_obj, register_name);
    soft_iic_stop(soft_iic_obj);

    soft_iic_start(soft_iic_obj);
    soft_iic_send_data(soft_iic_obj, soft_iic_obj->addr << 1 | 0x01);
    temp = soft_iic_read_data(soft_iic_obj, 1);
    soft_iic_stop(soft_iic_obj);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIC 接口初始化 默认 MASTER 模式 不提供 SLAVE 模式
// 参数说明     *soft_iic_obj   软件 IIC 指定信息存放结构体的指针
// 参数说明     addr            软件 IIC 地址 这里需要注意 标准七位地址 最高位忽略 写入时请务必确认无误
// 参数说明     delay           软件 IIC 延时 就是时钟高电平时间 越短 IIC 速率越高
// 参数说明     scl_pin         软件 IIC 时钟引脚号
// 参数说明     scl_port         软件 IIC 时钟引脚端口 
// 参数说明     sda_pin         软件 IIC 数据引脚号 
// 参数说明     sda_port         软件 IIC 数据引脚端口 
// 返回参数     void            
// 使用示例     soft_iic_init(&soft_iic_obj, addr, 100, B6, B7);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void soft_iic_init(soft_iic_info_struct *obj,
                   GPIO_TypeDef* scl_port, uint32_t scl_pin,
                   GPIO_TypeDef* sda_port, uint32_t sda_pin,
                   uint8_t addr, uint32_t delay)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    obj->scl_port = scl_port;
    obj->scl_pin  = scl_pin;
    obj->sda_port = sda_port;
    obj->sda_pin  = sda_pin;
    obj->addr     = addr;
    obj->delay    = delay;

    // === 配置 SCL 为推挽输出 ===
    GPIO_InitStruct.Pin = scl_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // 无上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    HAL_GPIO_Init(scl_port, &GPIO_InitStruct);

    // === 配置 SDA 为开漏输出（符合 I2C 规范） ===
    GPIO_InitStruct.Pin = sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;   // 开漏输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // 外部上拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(sda_port, &GPIO_InitStruct);

    // 默认空闲状态：SCL=1, SDA=1
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
}
