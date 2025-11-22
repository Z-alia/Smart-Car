#include "tof_uart_driver.h"
#include "uart_tpm.h"
#include "tof_timer.h"
#include "tof_cfg.h"
#include <stdio.h>

// Configuration
#define TOF_UART_CHANNEL_INDEX 0
#define TOF_UART_ID 0 // Just an ID passed to tx function

// Global variables
static uint16_t g_tof_distance = 0;
static uint8_t a_rxIsrBuff[50]; // Buffer for ISR reception
static uint8_t a_buff[60];      // Buffer for processing

// Forward declarations
void modbus_recv_data(uint8 *p_data, uint16 len);
uint8_t mcu_uart_tx_wrapper(uint8 channel, void *p_data, uint16 len);
void mcu_uart_init_wrapper(void);

// Configuration struct for uart_tpm
UART_TPM_ASP_CONFIG a_uartTpmAspConfig[] = 
{
	{TOF_UART_ID, a_rxIsrBuff, sizeof(a_rxIsrBuff), modbus_recv_data},
};

// Single byte buffer for interrupt reception
static uint8_t rx_byte;

void TOF_UART_Driver_Init(void)
{
	UART_TPM_CONFIG uartTpmConfig;
	
	uartTpmConfig.p_buff = a_buff;
	uartTpmConfig.buffLen = sizeof(a_buff);
	uartTpmConfig.framTimeInvt = 3; // 3ms frame timeout
	uartTpmConfig.pfun_mcuUartInit = mcu_uart_init_wrapper;
	uartTpmConfig.pfun_mcuUartTx = mcu_uart_tx_wrapper;
	uartTpmConfig.p_aspConfig = a_uartTpmAspConfig;
	uartTpmConfig.aspConfigNum = sizeof(a_uartTpmAspConfig)/sizeof(a_uartTpmAspConfig[0]);
	
	uart_tpm_init(&uartTpmConfig);
}

void mcu_uart_init_wrapper(void)
{
    // UART is already initialized by MX_UARTx_Init in main.c
    // We just need to start reception
    HAL_UART_Receive_IT(&TOF_UART_HANDLE, &rx_byte, 1);
}

uint8_t mcu_uart_tx_wrapper(uint8 channel, void *p_data, uint16 len)
{
    // channel is TOF_UART_ID, ignored here as we only use configured UART
    if (HAL_UART_Transmit(&TOF_UART_HANDLE, (uint8_t*)p_data, len, 100) == HAL_OK)
    {
        return 0;
    }
    return 1;
}

void TOF_UART_RxCpltCallback(void *huart)
{
    UART_HandleTypeDef *huart_ptr = (UART_HandleTypeDef *)huart;
    if (huart_ptr->Instance == TOF_UART_INSTANCE)
    {
        uart_tpm_rx_isr(TOF_UART_CHANNEL_INDEX, rx_byte);
        HAL_UART_Receive_IT(&TOF_UART_HANDLE, &rx_byte, 1);
    }
}

// Callback when a valid Modbus frame is received
void modbus_recv_data(uint8 *p_data, uint16 len)
{
    // p_data[0] = deviceAddr
    // p_data[1] = cmd
    // p_data[2] = byteNum
    // p_data[3] = dataH
    // p_data[4] = dataL
    
    if (len >= 5 && p_data[1] == 0x03) // Read Holding Registers response
    {
        uint16_t readData = p_data[3];
        readData <<= 8;
        readData |= p_data[4];
        g_tof_distance = readData;
    }
}

static uint8_t a_testBuff[20];

void TOF_UART_PeriodicRead(void)
{
	static uint32_t timStamp = 0;
	if(tim_check_timeout(timStamp, tim_get_count(), TOF_READ_PERIOD_MS)) // Read every period
	{
		a_testBuff[0] = TOF_DEV_ADDR; 
		a_testBuff[1] = 0x03; // Read Holding Registers
		a_testBuff[2] = TOF_REG_ADDR>>8;
		a_testBuff[3] = TOF_REG_ADDR;
		a_testBuff[4] = 0;
		a_testBuff[5] = 1; // Read 1 register
		uart_tpm_tx_data(TOF_UART_CHANNEL_INDEX, a_testBuff, 6);
		timStamp = tim_get_count();
	}
}

void TOF_UART_Driver_Task(void)
{
    TOF_UART_PeriodicRead();
    uart_tpm_main();
}

uint16_t TOF_UART_GetDistance(void)
{
    return g_tof_distance;
}
