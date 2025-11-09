/**
  ******************************************************************************
  * @file    tof.c
  * @author  kevin_guo
  * @version V2.0.0
  * @date    2025-11-09
  * @brief   TOF sensor I2C driver for STM32H750
  ******************************************************************************
  * @attention
	* 硬件连接�? 
	*           PB8  - TOF_SCL      
  *           PB9  - TOF_SDA   
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "tof.h"

/* Private function prototypes -----------------------------------------------*/
static void TOF_DelayuS(unsigned int nCount);
static void TOF_Start(void);
static void TOF_Stop(void);
static unsigned char TOF_Wait_Ack(void);
static void TOF_ack(void);
static void TOF_nack(void);
static void TOF_Send_Byte(unsigned char dat);
static unsigned char TOF_Read_Byte(void);
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : TOF_DelayuS
* Description    : Delay function for I2C timing
* Input          : nCount: delay time
* Output         : None
* Return         : None
*******************************************************************************/
static void TOF_DelayuS(unsigned int nCount)
{
  // STM32H750运行频率480MHz，需要更多NOP指令来实现相同延�?
  // 根据实际时钟频率调整，这里按480MHz计算
  for(; nCount != 0; nCount--)
	{//100kHz I2C时序
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}
}
/*******************************************************************************
* Function Name  : TOF_Start
* Description    : I2C start signal
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void TOF_Start(void)
{
	TOF_SDA_OUT();     // 确保SDA为输出模�?
	TOF_SDA_HIGH();	   // SDA拉高
	TOF_SCL_HIGH();    // SCL拉高
	TOF_DelayuS(4);
 	TOF_SDA_LOW();     // SDA拉低（起始信号）
	TOF_DelayuS(4);
	TOF_SCL_LOW();     // SCL拉低，准备发送数�?
	TOF_DelayuS(2);
}	  
/*******************************************************************************
* Function Name  : TOF_Stop
* Description    : I2C停止信号：SCL为高时，SDA由低变高
* Input          : None
* Output         : None
* Return         : None
* Return         : None
*******************************************************************************/
static void TOF_Stop(void)
{
	TOF_SDA_OUT();     // 确保SDA为输出模�?
	TOF_SCL_LOW();
	TOF_SDA_LOW();     // SDA先拉�?
 	TOF_DelayuS(4);
	TOF_SCL_HIGH();    // SCL拉高
 	TOF_DelayuS(4);
	TOF_SDA_HIGH();    // SDA拉高（停止信号）
	TOF_DelayuS(4);
}
/*******************************************************************************
* Function Name  : TOF_Wait_Ack
* Description    : �?个时钟上升沿读取ack信号
* Input          : None
* Output         : None
* Return         : =0有ack
*								 : =1无ack
*******************************************************************************/
static unsigned char TOF_Wait_Ack(void)
{
	unsigned int ucErrTime;
  unsigned char RetValue = 1;  // 默认无应�?
	
	TOF_SDA_IN();      //SDA设置为输�? 
	TOF_SDA_HIGH();    // 释放SDA�?
	TOF_DelayuS(2);
	TOF_SCL_LOW();
 	TOF_DelayuS(2);
	TOF_SCL_HIGH();    // 时钟拉高，准备读取ACK
	TOF_DelayuS(2);
	
  ucErrTime = 10000;
  while(ucErrTime-- > 0)
  {
    if(TOF_READ_SDA() == 0)  // 读到低电�?有ACK
    {
      RetValue = 0;
			break;
    }
  }
	
	TOF_SCL_LOW();     // 时钟拉低
 	TOF_DelayuS(2);	
	TOF_SDA_OUT();     // SDA恢复为输�?
	return RetValue;  
} 
/*******************************************************************************
* Function Name  : TOF_ack
* Description    : �?个时钟上升沿发送低电平ACK
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void TOF_ack(void)
{
	TOF_SDA_OUT();     // 确保SDA为输出模�?
	TOF_SCL_LOW();
	TOF_SDA_LOW();     // 发送ACK（低电平�?
	TOF_DelayuS(2);
	TOF_SCL_HIGH();
	TOF_DelayuS(5);
	TOF_SCL_LOW();
	TOF_DelayuS(2);
}

/*******************************************************************************
* Function Name  : TOF_nack
* Description    : �?个时钟上升沿发送高电平NACK
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/   
static void TOF_nack(void)
{
	TOF_SDA_OUT();     // 确保SDA为输出模�?
	TOF_SCL_LOW();
	TOF_SDA_HIGH();    // 发送NACK（高电平�?
	TOF_DelayuS(2);
	TOF_SCL_HIGH();
	TOF_DelayuS(5);
	TOF_SCL_LOW();
	TOF_DelayuS(2);
}					
 
/*******************************************************************************
* Function Name  : TOF_Send_Byte
* Description    : �?字节数据到i2c总线
* Input          : dat-要发送的数据
* Output         : None
* Return         : None
*******************************************************************************/	  
static void TOF_Send_Byte(unsigned char dat)
{                        
	unsigned char i; 
	
	TOF_SDA_OUT();     // 确保SDA为输出模�?
	TOF_SCL_LOW(); 	 
	TOF_DelayuS(2); 
	
	for(i=0; i<8; i++)
	{         
		// 发送数据位（MSB first�?
		if(dat & 0x80)
			TOF_SDA_HIGH();
		else
			TOF_SDA_LOW();
		
		TOF_DelayuS(2);
		TOF_SCL_HIGH();    // 时钟拉高，从机读取数�?
		TOF_DelayuS(5);
		TOF_SCL_LOW();     // 时钟拉低
		TOF_DelayuS(2);
		
		dat <<= 1;         // 左移准备下一�?
	}	 
} 	 
/*******************************************************************************
* Function Name  : TOF_Read_Byte
* Description    : 从i2c总线�?字节数据
* Input          : None
* Output         : None
* Return         : 读到�?字节数据
*******************************************************************************/ 
static unsigned char TOF_Read_Byte(void)
{
	unsigned char i, receive = 0;
	
	TOF_SDA_IN();      // SDA设置为输入模�?
	TOF_SDA_HIGH();    // 释放SDA�?
	TOF_DelayuS(2);
	
	for(i=0; i<8; i++)
	{ 
		receive <<= 1;     // 左移准备接收下一�?
		
		TOF_SCL_LOW();
		TOF_DelayuS(2);
		TOF_SCL_HIGH();    // 时钟拉高，读取数�?
		TOF_DelayuS(2);
		
		if(TOF_READ_SDA())
			receive |= 0x01;  // 读到高电�?
		
		TOF_DelayuS(3);
	}
	
	TOF_SCL_LOW();
	TOF_DelayuS(2);
	TOF_SDA_OUT();     // SDA恢复为输出模�?
	
	return receive;
}

/*******************************************************************************
* Function Name  : TOF_WriteNByte
* Description    : Sensor read api.
* Input          : *txbuff-the buffer which to write.
*                : regaddr-the address write to
*                : size-write data size
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char TOF_WriteNByte(unsigned char *txbuff, unsigned char regaddr, unsigned char size)
{
	unsigned char i = 0;
  
	TOF_Start();

  TOF_Send_Byte( TOF_ADDR|0x00 );
  if( 1 == TOF_Wait_Ack() )
  {
		TOF_Stop();
    return 1;
  }
  TOF_Send_Byte( regaddr&0xff );
  if( 1 == TOF_Wait_Ack() )
  {
		TOF_Stop();
    return 1;
  }
	for ( i = 0; i < size; i++)
	{
		TOF_Send_Byte( txbuff[size-i-1] );
		if( 1 == TOF_Wait_Ack() )
		{
			TOF_Stop();
			return 1;
		}
	}
  TOF_Stop();
	
  return 0;
}
/*******************************************************************************
* Function Name  : TOF_ReadNByte
* Description    : Sensor read api.
* Input          : *rxbuff-the buffer which stores read content.
*                : regaddr-the address read from
*                : size-read data size
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char TOF_ReadNByte(unsigned char *rxbuff, unsigned char regaddr, unsigned char size)
{
  unsigned char i = 0;
	
  TOF_Start();

  TOF_Send_Byte( TOF_ADDR|0x00 );
  if( 1 == TOF_Wait_Ack() )
  {
		TOF_Stop();
    return 1;
  }
  TOF_Send_Byte( regaddr&0xff );
  if( 1 == TOF_Wait_Ack() )
  {
		TOF_Stop();
    return 1;
  }
	TOF_Stop();
  TOF_Start();
  TOF_Send_Byte( TOF_ADDR|0x01 );
  if( 1 == TOF_Wait_Ack() )
  {
		TOF_Stop();
    return 1;
  }
	TOF_DelayuS(30);//30uS
	for ( i = 0; i < size; i++)
	{
		rxbuff[size-i-1] = TOF_Read_Byte();	
		if((i+1)==size)			
			TOF_nack();
		else
			TOF_ack();
		TOF_DelayuS(30);//30uS
	}
  TOF_Stop();
	
  return 0;
}
/*******************************************************************************
* Function Name  : TOF_Init
* Description    : config i2c driver gpio
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TOF_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 使能GPIO时钟
	TOF_GPIO_CLK_ENABLE();
	   
	// 配置GPIO为开漏输出模式，适合I2C总线
	GPIO_InitStructure.Pin = TOF_SCL_PIN | TOF_SDA_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输�?
	GPIO_InitStructure.Pull = GPIO_PULLUP;          // 上拉
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH; // 高�?
	HAL_GPIO_Init(TOF_GPIO_PORT, &GPIO_InitStructure);
	
	// 设置初始状态为高电�?
	HAL_GPIO_WritePin(TOF_GPIO_PORT, TOF_SCL_PIN | TOF_SDA_PIN, GPIO_PIN_SET);
}
/*******************************************************************************
* Function Name  : TOF_ReadDistance
* Description    : Read real-time distance from TOF sensor
* Input          : None
* Output         : None
* Return         : Distance in millimeters (0 = read failed or invalid)
*******************************************************************************/
unsigned short TOF_ReadDistance(void)
{
	unsigned short distance = 0;
	
	// 从寄存器0x00读取2字节实时距离数据
	if(TOF_ReadNByte((unsigned char*)&distance, TOF_REG_DISTANCE_REAL, 2) == 0)
	{
		// 检查距离是否在有效范围�?(10mm-1800mm)
		if(distance >= 10 && distance <= 1800)
		{
			return distance;
		}
	}
	
	return 0;  // 读取失败或超出范围返�?
}
/*******************************************************************************
* Function Name  : TOF_ReadDistanceFiltered
* Description    : Read filtered distance from TOF sensor
* Input          : None
* Output         : None
* Return         : Distance in millimeters (0 = read failed or invalid)
*******************************************************************************/
unsigned short TOF_ReadDistanceFiltered(void)
{
	unsigned int distance = 0;  // 4字节滤波距离
	
	// 从寄存器0x02读取4字节滤波距离数据
	if(TOF_ReadNByte((unsigned char*)&distance, TOF_REG_DISTANCE_FILTER, 4) == 0)
	{
		// 检查距离是否在有效范围�?(10mm-1800mm)
		if(distance >= 10 && distance <= 1800)
		{
			return (unsigned short)distance;
		}
	}
	
	return 0;  // 读取失败或超出范围返�?
}
/*******************************************************************************
* Function Name  : TOF_SetOutputMode
* Description    : Set TOF output mode
* Input          : mode - 0: filtered value, 1: real-time value
* Output         : None
* Return         : 0=success, 1=failed
*******************************************************************************/
unsigned char TOF_SetOutputMode(unsigned char mode)
{
	if(mode > 1) return 1;  // 参数检�?
	
	return TOF_WriteNByte(&mode, TOF_REG_OUTPUT_MODE, 1);
}
/*******************************************************************************
* Function Name  : TOF_SetTriggerMode
* Description    : Set TOF trigger mode
* Input          : mode - 0: continuous auto-send, 1: host read trigger
* Output         : None
* Return         : 0=success, 1=failed
*******************************************************************************/
unsigned char TOF_SetTriggerMode(unsigned char mode)
{
	if(mode > 1) return 1;  // 参数检�?
	
	return TOF_WriteNByte(&mode, TOF_REG_TRIGGER_MODE, 1);
}
/*******************************************************************************
* Function Name  : TOF_SetOffset
* Description    : Set TOF offset calibration
* Input          : offset - offset value in mm (-99 to 99)
* Output         : None
* Return         : 0=success, 1=failed
*******************************************************************************/
unsigned char TOF_SetOffset(short offset)
{
	if(offset < -99 || offset > 99) return 1;  // 参数检�?
	
	return TOF_WriteNByte((unsigned char*)&offset, TOF_REG_OFFSET, 2);
}
/*******************************************************************************
* Function Name  : TOF_SetXtalk
* Description    : Set TOF xtalk calibration
* Input          : xtalk - xtalk value (0-200)
* Output         : None
* Return         : 0=success, 1=failed
*******************************************************************************/
unsigned char TOF_SetXtalk(unsigned char xtalk)
{
	if(xtalk > 200) return 1;  // 参数检�?
	
	return TOF_WriteNByte(&xtalk, TOF_REG_XTALK, 1);
}
/*******************************************************************************
* Function Name  : TOF_SetIntervalTime
* Description    : Set TOF interval time for distance detection
* Input          : time_ms - interval time in milliseconds (10-9999)
* Output         : None
* Return         : 0=success, 1=failed
*******************************************************************************/
unsigned char TOF_SetIntervalTime(unsigned short time_ms)
{
	if(time_ms < 10 || time_ms > 9999) return 1;  // 参数检�?
	
	return TOF_WriteNByte((unsigned char*)&time_ms, TOF_REG_INTERVAL_TIME, 2);
}
/*******************************************************************************
* Function Name  : TOF_SetI2CAddress
* Description    : Set TOF I2C slave address (will take effect after power cycle)
* Input          : new_addr - new I2C address (0x0001-0x00FE)
* Output         : None
* Return         : 0=success, 1=failed
*******************************************************************************/
unsigned char TOF_SetI2CAddress(unsigned short new_addr)
{
	if(new_addr < 0x0001 || new_addr > 0x00FE) return 1;  // 参数检�?
	
	return TOF_WriteNByte((unsigned char*)&new_addr, TOF_REG_I2C_SLAVE_ID, 2);
}
