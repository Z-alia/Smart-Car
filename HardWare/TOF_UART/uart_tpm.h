#ifndef __UART_TPM_H_
#define __UART_TPM_H_
#include "tof_type.h"

#define UART_TPM_ERR_RXFLOW		0x0001
#define UART_TPM_ERR_UNPACKCX	0x0100
#define UART_TPM_ERR_UNPACKCDYC	0x0200
#define UART_TPM_ERR_UNPACKCHK	0x0400
#define UART_TPM_ERR_UNPACKZY	0x0800

#define UART_TPM_ERR_UNPACKFLOW	0x0002
#define UART_TPM_ERR_PACKFLOW	0x0004


#pragma pack(1)
typedef struct
{
	uint8		uartId;//channel id
	uint8		*p_isrRx;
	uint16		isrRxLen;
	void		(*pfun_recvCallback)(uint8 *, uint16);
	
	uint32		lastRxTimestamp;
	uint16		rxWrIndex;
	uint16		rxRdIndex;
	uint32		errStatus;
}UART_TPM_ASP_CONFIG;
#pragma pack()

#pragma pack(1)
typedef struct
{
	uint8		*p_buff;					
	uint16		buffLen;
	uint8		framTimeInvt;				
	void		(*pfun_mcuUartInit)(void);
	uint8		(*pfun_mcuUartTx)(uint8, void *, uint16);
	UART_TPM_ASP_CONFIG *p_aspConfig;
	uint8		aspConfigNum;
}UART_TPM_CONFIG;
#pragma pack()

void uart_tpm_init(UART_TPM_CONFIG *p_Config);
void uart_tpm_main(void);
uint8 uart_tpm_tx_data(uint8 channel, uint8 *p_data, uint16 len);
uint8 uart_tpm_rx_isr(uint8 channel, uint8 data);

#endif//__UART_TPM_H_
