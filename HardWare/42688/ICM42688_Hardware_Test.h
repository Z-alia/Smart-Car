/**
 * @file ICM42688_Hardware_Test.h
 * @brief ICM-42688P 硬件层诊断头文件
 */

#ifndef ICM42688_HARDWARE_TEST_H
#define ICM42688_HARDWARE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 测试函数声明 */

/**
 * @brief 测试 MISO 引脚是否能接收数据
 */
void Test_MISO_Line(void);

/**
 * @brief 测试 ICM-42688P 供电和连接
 */
void Test_ICM_Power_And_Connection(void);

/**
 * @brief 尝试不同的 SPI 时钟极性和相位
 */
void Test_SPI_Modes(void);

/**
 * @brief 主诊断函数 - 按顺序运行所有测试
 */
void ICM42688_Full_Hardware_Diagnostic(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_HARDWARE_TEST_H */
