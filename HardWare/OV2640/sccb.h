#ifndef __CAMERA_SCCB_H
#define __CAMERA_SCCB_H

#include "main.h"

#define OV2640_DEVICE_ADDRESS     0x60    // OV2640��ַ
#define OV5640_DEVICE_ADDRESS     0X78		// OV5640��ַ

/*----------------------------------------- IIIC �������ú� -----------------------------------------------*/

#define SCCB_SCL_CLK_ENABLE       __HAL_RCC_GPIOB_CLK_ENABLE()		// SCL ����ʱ��
#define SCCB_SCL_PORT   		   SCCB_SCL_GPIO_Port               // SCL ���Ŷ˿�
#define SCCB_SCL_PIN     		   SCCB_SCL_Pin 					// SCL ����
        
#define SCCB_SDA_CLK_ENABLE       __HAL_RCC_GPIOB_CLK_ENABLE() 	// SDA ����ʱ��
#define SCCB_SDA_PORT   			 SCCB_SDA_GPIO_Port                   	// SDA ���Ŷ˿�
#define SCCB_SDA_PIN    			 SCCB_SDA_Pin              	// SDA ����

/*------------------------------------------ IIC��ض��� -------------------------------------------------*/

#define ACK_OK  	1  			// ��Ӧ����
#define ACK_ERR 	0				// ��Ӧ����

// SCCBͨ����ʱ��SCCB_Delay()����ʹ�ã�
//	ͨ���ٶ���300KHz����
// 增加延时值从8到32，降低通信速度以提高稳定性
#define SCCB_DelayVaule  32
  	

/*-------------------------------------------- IO�ڲ��� ---------------------------------------------------*/   

#define SCCB_SCL(a)	if (a)	\
										HAL_GPIO_WritePin(SCCB_SCL_PORT, SCCB_SCL_PIN, GPIO_PIN_SET); \
									else		\
										HAL_GPIO_WritePin(SCCB_SCL_PORT, SCCB_SCL_PIN, GPIO_PIN_RESET)	

#define SCCB_SDA(a)	if (a)	\
										HAL_GPIO_WritePin(SCCB_SDA_PORT, SCCB_SDA_PIN, GPIO_PIN_SET); \
									else		\
										HAL_GPIO_WritePin(SCCB_SDA_PORT, SCCB_SDA_PIN, GPIO_PIN_RESET)		

// SDA方向切换宏 - 简化版本，开漏输出模式下不需要频繁切换
// 开漏输出高电平时引脚呈高阻态，可以直接读取
#define SCCB_SDA_IN()  SCCB_SDA(1)  // 输出高电平，释放总线

#define SCCB_SDA_OUT() // 空操作，始终保持开漏输出模式		

/*--------------------------------------------- �������� --------------------------------------------------*/  		
					
void 		SCCB_GPIO_Config (void);				// IIC���ų�ʼ��
void 		SCCB_Delay(uint32_t a);					// IIC��ʱ����						
void 		SCCB_Start(void);							// ����IICͨ��
void 		SCCB_Stop(void);							// IICֹͻ�ź�
void 		SCCB_ACK(void);							//	������Ӧ�ź�
void 		SCCB_NoACK(void);							// ���ͷ�Ӧ���ź�
uint8_t 	SCCB_WaitACK(void);						//	�ȴ�Ӧ���ź�
uint8_t	SCCB_WriteByte(uint8_t IIC_Data); 	// д�ֽں���
uint8_t 	SCCB_ReadByte(uint8_t ACK_Mode);		// ���ֽں���
uint8_t  SCCB_Test_Bus(void);                  // 测试SCCB总线状态
		
uint8_t  SCCB_WriteReg (uint8_t addr,uint8_t value);     	// ��ָ���ļĴ���(8λ��ַ)дһ�ֽ����ݣ�OV2640�õ�
uint8_t  SCCB_ReadReg (uint8_t addr);                    	// ��ָ���ļĴ���(8λ��ַ)��һ�ֽ����ݣ�OV2640�õ�
									
uint8_t 	SCCB_WriteReg_16Bit(uint16_t addr,uint8_t value);	// ��ָ���ļĴ���(16λ��ַ)дһ�ֽ����ݣ�OV5640�õ�									
uint8_t 	SCCB_ReadReg_16Bit (uint16_t addr);						// ��ָ���ļĴ���(16λ��ַ)��һ�ֽ����ݣ�OV5640�õ�
uint8_t 	SCCB_WriteBuffer_16Bit(uint16_t addr,uint8_t *pData, uint32_t size);	// ��ָ���ļĴ���(16λ��ַ)����д���ݣ�OV5640 д���Զ��Խ��̼�ʱ�õ�		
									
#endif //__CAMERA_SCCB_H
