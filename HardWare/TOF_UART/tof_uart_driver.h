#ifndef __TOF_UART_DRIVER_H_
#define __TOF_UART_DRIVER_H_

#include "tof_type.h"

// Initialize the TOF UART driver
void TOF_UART_Driver_Init(void);

// Main loop task for TOF UART driver
void TOF_UART_Driver_Task(void);

// Call this from HAL_UART_RxCpltCallback in main.c or usart.c
void TOF_UART_RxCpltCallback(void *huart);

// Function to get the latest distance reading
uint16_t TOF_UART_GetDistance(void);

#endif // __TOF_UART_DRIVER_H_
