#include "soft_iic.h"
#include "i2c.h"
#include "mpu6050.h"

int16_t mpu6050_gyro_x = 0, mpu6050_gyro_y = 0, mpu6050_gyro_z = 0;               // 三轴陀螺仪数据      gyro (陀螺仪)
int16_t mpu6050_acc_x = 0, mpu6050_acc_y = 0, mpu6050_acc_z = 0;                  // 三轴加速度计数据    acc (accelerometer 加速度计)

static soft_iic_info_struct mpu6050_iic_struct;

#if MPU6050_USE_SOFT_IIC                                       // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
#define mpu6050_write_register(reg, data)       (soft_iic_write_8bit_register(&mpu6050_iic_struct, (reg), (data)))
#define mpu6050_read_register(reg)              (soft_iic_read_8bit_register(&mpu6050_iic_struct, (reg)))
#define mpu6050_read_registers(reg, data, len)  (soft_iic_read_8bit_registers(&mpu6050_iic_struct, (reg), (data), (len)))
#else
static I2C_HandleTypeDef *g_pHI2C_MPU6050 = &hi2c2;	//硬件I2C句柄

static int mpu6050_write_register(uint8_t reg, uint8_t data)
{
    uint8_t tmpbuf[2];

    tmpbuf[0] = reg;
    tmpbuf[1] = data;
    
	return HAL_I2C_Master_Transmit(g_pHI2C_MPU6050, MPU6050_DEV_ADDR<<1, tmpbuf, 2, MPU6050_TIMEOUT_COUNT);
}

int mpu6050_read_register(uint8_t reg, uint8_t* pdata)
{
	return HAL_I2C_Mem_Read(g_pHI2C_MPU6050, MPU6050_DEV_ADDR<<1, reg, 1, pdata, 1, MPU6050_TIMEOUT_COUNT);
}

int mpu6050_read_registers(uint8_t reg, uint8_t *pdata,uint8_t len)
{
	return HAL_I2C_Mem_Read(g_pHI2C_MPU6050, MPU6050_DEV_ADDR<<1, reg, 1, pdata, len, MPU6050_TIMEOUT_COUNT);
}

#endif
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MPU6050 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     if(mpu6050_self1_check())
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t mpu6050_self1_check (void)
{
    uint8_t dat = 0, return_state = 0;
    uint16_t timeout_count = 0;

    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);                           // 解除休眠状态
    mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07);                           // 125HZ采样率
    while(0x07 != dat)
    {
        if(MPU6050_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state =  1;
            break;
        }
		#if MPU6050_USE_SOFT_IIC 
        dat = mpu6050_read_register(MPU6050_SMPLRT_DIV);
		#else 
		mpu6050_read_register(MPU6050_SMPLRT_DIV,&dat);
		#endif
        HAL_Delay(10);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 MPU6050 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     mpu6050_get_acc();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_get_acc (void)
{
    uint8_t dat[6];

    mpu6050_read_registers(MPU6050_ACCEL_XOUT_H, dat, 6);  
    mpu6050_acc_x = (int16_t)(((uint16_t)dat[0] << 8 | dat[1]));
    mpu6050_acc_y = (int16_t)(((uint16_t)dat[2] << 8 | dat[3]));
    mpu6050_acc_z = (int16_t)(((uint16_t)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 MPU6050 陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     mpu6050_get_gyro();                                             // 执行该函数后，直接查看对应的变量即可
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_get_gyro (void)
{
    uint8_t dat[6];

    mpu6050_read_registers(MPU6050_GYRO_XOUT_H, dat, 6);  
    mpu6050_gyro_x = (int16_t)(((uint16_t)dat[0] << 8 | dat[1]));
    mpu6050_gyro_y = (int16_t)(((uint16_t)dat[2] << 8 | dat[3]));
    mpu6050_gyro_z = (int16_t)(((uint16_t)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 MPU6050 加速度计数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的加速度计数据
// 返回参数     void
// 使用示例     float data = mpu6050_acc_transition(mpu6050_acc_x);                // 单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float mpu6050_acc_transition (int16_t acc_value)
{
    float acc_data = 0;
    switch(MPU6050_ACC_SAMPLE)
    {
        case 0x00: acc_data = (float)acc_value / 16384; break;                  // 0x00 加速度计量程为:±2 g    获取到的加速度计数据 除以 16384      可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
        case 0x08: acc_data = (float)acc_value / 8192;  break;                  // 0x08 加速度计量程为:±4 g    获取到的加速度计数据 除以 8192       可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
        case 0x10: acc_data = (float)acc_value / 4096;  break;                  // 0x10 加速度计量程为:±8 g    获取到的加速度计数据 除以 4096       可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
        case 0x18: acc_data = (float)acc_value / 2048;  break;                  // 0x18 加速度计量程为:±16g    获取到的加速度计数据 除以 2048       可以转化为带物理单位的数据 (g 代表重力加速度 物理学名词 一般情况下 g 取 9.8 m/s^2 为标准值)
        default: break;
    }
    return acc_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 MPU6050 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float data = mpu6050_gyro_transition(mpu6050_gyro_x);           // 单位为°/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float mpu6050_gyro_transition (int16_t gyro_value)
{
    float gyro_data = 0;
    switch(MPU6050_GYR_SAMPLE)
    {
        case 0x00: gyro_data = (float)gyro_value / 131.0f;  break;              // 0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以 131           可以转化为带物理单位的数据，单位为：°/s
        case 0x08: gyro_data = (float)gyro_value / 65.5f;   break;              // 0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以 65.5          可以转化为带物理单位的数据，单位为：°/s
        case 0x10: gyro_data = (float)gyro_value / 32.8f;   break;              // 0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以 32.8          可以转化为带物理单位的数据，单位为：°/s
        case 0x18: gyro_data = (float)gyro_value / 16.4f;   break;              // 0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以 16.4          可以转化为带物理单位的数据，单位为：°/s
        default: break;
    }
    return gyro_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 MPU6050
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     mpu6050_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050_init (void)
{
    uint8_t return_state = 0;
	#if MPU6050_USE_SOFT_IIC
    soft_iic_init(&mpu6050_iic_struct, MPU6050_SCL_PORT, MPU6050_SCL_PIN,MPU6050_SDA_PORT,
	MPU6050_SDA_PIN,MPU6050_DEV_ADDR, MPU6050_SOFT_IIC_DELAY);//软件I2C初始化
    HAL_Delay(100);// 上电延时  
	#else
	MX_I2C2_Init();//硬件I2C初始化
	HAL_Delay(100);                                                       // 上电延时
	#endif
    do
    {
        if(mpu6050_self1_check())
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 MPU6050 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            return_state = 1;
            break;
        }
        mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);                       // 解除休眠状态
        mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07);                       // 125HZ采样率
        mpu6050_write_register(MPU6050_CONFIG, 0x04);

        mpu6050_write_register(MPU6050_GYRO_CONFIG, MPU6050_GYR_SAMPLE);        // 2000
        // GYRO_CONFIG寄存器
        // 设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131.2         可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.6          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

        mpu6050_write_register(MPU6050_ACCEL_CONFIG, MPU6050_ACC_SAMPLE);       // 8g
        // ACCEL_CONFIG寄存器
        // 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)

        mpu6050_write_register(MPU6050_USER_CONTROL, 0x00);
        mpu6050_write_register(MPU6050_INT_PIN_CFG, 0x02);
    }while(0);
    return return_state;
}
