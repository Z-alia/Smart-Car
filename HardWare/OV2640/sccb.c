/***
	************************************************************************************************
	*	@file  	sccb_2640.c
	*	@version V1.0
	*  @date    2021-12-13
	*	@author  ���ͿƼ�	
	*	@brief   SCCB �ӿ���غ���������ģ��IIC����ʽ��
   *************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32H750VBT6���İ� ���ͺţ�FK750M1-VBT6��
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*	
>>>>>	�ļ�˵����
	*
	*  1.SCCB��غ�����SCCB��IICһ����
	* 	2.ʹ��ģ��ӿڵ���ʽ
	*	3.ͨ���ٶ�Ĭ��Ϊ 300KHz ���ң�����ܳ���400K
	*
	************************************************************************************************************************
***/

#include "sccb.h"  


/*****************************************************************************************
*	�� �� ��: SCCB_GPIO_Config
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: ��ʼ��IIC��GPIO��,�������
*	˵    ��: ����IICͨ���ٶȲ��ߣ������IO���ٶ�����Ϊ2M����
******************************************************************************************/

void SCCB_GPIO_Config (void)
{
	SCCB_SDA_OUT();  // 确保SDA为输出模式
	HAL_GPIO_WritePin(SCCB_SCL_PORT, SCCB_SCL_PIN, GPIO_PIN_SET);		// SCL����ߵ�ƽ
	HAL_GPIO_WritePin(SCCB_SDA_PORT, SCCB_SDA_PIN, GPIO_PIN_SET);    // SDA����ߵ�ƽ
	SCCB_Delay(SCCB_DelayVaule * 10);  // 初始化后延时
}

/*****************************************************************************************
*	�� �� ��: SCCB_Delay
*	��ڲ���: a - ��ʱʱ��
*	�� �� ֵ: ��
*	��������: ����ʱ����
*	˵    ��: Ϊ����ֲ�ļ�����Ҷ���ʱ����Ҫ�󲻸ߣ����Բ���Ҫʹ�ö�ʱ������ʱ
******************************************************************************************/

void SCCB_Delay(uint32_t a)
{
	volatile uint16_t i;
	while (a --)				
	{
		for (i = 0; i < 6; i++);
	}
}

/*****************************************************************************************
*	�� �� ��: SCCB_Test_Bus
*	��ڲ���: ��
*	�� �� ֵ: 0-��������, 1-SCL����, 2-SDA����, 3-������
*	��������: ����SCCB��������
*	˵    ��: ���SCL��SDA�����Ƿ���������߲�����
******************************************************************************************/

uint8_t SCCB_Test_Bus(void)
{
	uint8_t scl_state, sda_state;
	
	// 设置SCL和SDA为高电平
	HAL_GPIO_WritePin(SCCB_SCL_PORT, SCCB_SCL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SCCB_SDA_PORT, SCCB_SDA_PIN, GPIO_PIN_SET);
	SCCB_Delay(SCCB_DelayVaule * 10);
	
	// 读取状态
	scl_state = HAL_GPIO_ReadPin(SCCB_SCL_PORT, SCCB_SCL_PIN);
	sda_state = HAL_GPIO_ReadPin(SCCB_SDA_PORT, SCCB_SDA_PIN);
	
	// 返回状态：bit0=SDA, bit1=SCL
	return (scl_state << 1) | sda_state;
}

/*****************************************************************************************
*	�� �� ��: SCCB_Start
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: IIC��ʼ�ź�
*	˵    ��: ��SCL���ڸߵ�ƽ�ڼ䣬SDA�ɸߵ�������Ϊ��ʼ�ź�
******************************************************************************************/

void SCCB_Start(void)
{
	SCCB_SDA_OUT();  // 确保SDA为输出模式
	SCCB_SDA(1);		
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);
	
	SCCB_SDA(0);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SCL(0);
	SCCB_Delay(SCCB_DelayVaule);
}

/*****************************************************************************************
*	�� �� ��: SCCB_Stop
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: IICֹͣ�ź�
*	˵    ��: ��SCL���ڸߵ�ƽ�ڼ䣬SDA�ɵ͵�������Ϊ��ʼ�ź�
******************************************************************************************/

void SCCB_Stop(void)
{
	SCCB_SCL(0);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SDA(0);
	SCCB_Delay(SCCB_DelayVaule);
	
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SDA(1);
	SCCB_Delay(SCCB_DelayVaule);
}

/*****************************************************************************************
*	�� �� ��: SCCB_ACK
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: IICӦ���ź�
*	˵    ��: ��SCLΪ�ߵ�ƽ�ڼ䣬SDA�������Ϊ�͵�ƽ������Ӧ���ź�
******************************************************************************************/

void SCCB_ACK(void)
{
	SCCB_SCL(0);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SDA(0);
	SCCB_Delay(SCCB_DelayVaule);	
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);
	
	SCCB_SCL(0);		// SCL�����ʱ��SDAӦ�������ߣ��ͷ�����
	SCCB_SDA(1);		
	
	SCCB_Delay(SCCB_DelayVaule);

}

/*****************************************************************************************
*	�� �� ��: SCCB_NoACK
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: IIC��Ӧ���ź�
*	˵    ��: ��SCLΪ�ߵ�ƽ�ڼ䣬��SDA����Ϊ�ߵ�ƽ��������Ӧ���ź�
******************************************************************************************/

void SCCB_NoACK(void)
{
	SCCB_SCL(0);	
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SDA(1);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);
	
	SCCB_SCL(0);
	SCCB_Delay(SCCB_DelayVaule);
}

/*****************************************************************************************
*	�� �� ��: SCCB_WaitACK
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: �ȴ������豸����Ӧ���ź�
*	˵    ��: ��SCLΪ�ߵ�ƽ�ڼ䣬����⵽SDA����Ϊ�͵�ƽ��������豸��Ӧ����
******************************************************************************************/

uint8_t SCCB_WaitACK(void)
{
	SCCB_SDA_IN();   // 切换为输入模式以读取ACK
	SCCB_SDA(1);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);	
	
	if( HAL_GPIO_ReadPin(SCCB_SDA_PORT,SCCB_SDA_PIN) != 0) //�ж��豸�Ƿ���������Ӧ		
	{
		SCCB_SCL(0);	
		SCCB_Delay( SCCB_DelayVaule );	
		SCCB_SDA_OUT();  // 切换回输出模式
		return ACK_ERR;	//��Ӧ��
	}
	else
	{
		SCCB_SCL(0);	
		SCCB_Delay( SCCB_DelayVaule );	
		SCCB_SDA_OUT();  // 切换回输出模式
		return ACK_OK;	//Ӧ������
	}
}

/*****************************************************************************************
*	�� �� ��:	SCCB_WriteByte
*	��ڲ���:	IIC_Data - Ҫд���8λ����
*	�� �� ֵ:	ACK_OK  - �豸��Ӧ����
*          	   ACK_ERR - �豸��Ӧ����
*	��������:	дһ�ֽ�����
*	˵    ��:   ��λ��ǰ
******************************************************************************************/

uint8_t SCCB_WriteByte(uint8_t IIC_Data)
{
	uint8_t i;
	
	SCCB_SDA_OUT();  // 确保SDA为输出模式

	for (i = 0; i < 8; i++)
	{
		SCCB_SDA(IIC_Data & 0x80);
		
		SCCB_Delay( SCCB_DelayVaule );
		SCCB_SCL(1);
		SCCB_Delay( SCCB_DelayVaule );
		SCCB_SCL(0);		
		if(i == 7)
		{
			SCCB_SDA(1);			
		}
		IIC_Data <<= 1;
	}

	return SCCB_WaitACK(); //�ȴ��豸��Ӧ
}

/*****************************************************************************************
*	�� �� ��:	SCCB_ReadByte
*	��ڲ���:	ACK_Mode - ��Ӧģʽ������1�򷢳�Ӧ���źţ�����0������Ӧ���ź�
*	�� �� ֵ:	ACK_OK  - �豸��Ӧ����
*          	   ACK_ERR - �豸��Ӧ����
*	��������:   ��һ�ֽ�����
*	˵    ��:   1.��λ��ǰ
*				   2.Ӧ�������������һ�ֽ�����ʱ���ͷ�Ӧ���ź�
******************************************************************************************/

uint8_t SCCB_ReadByte(uint8_t ACK_Mode)
{
	uint8_t IIC_Data = 0;
	uint8_t i = 0;
	
	SCCB_SDA_IN();   // 切换为输入模式以读取数据
	
	for (i = 0; i < 8; i++)
	{
		IIC_Data <<= 1;
		
		SCCB_SCL(1);
		SCCB_Delay( SCCB_DelayVaule );
		IIC_Data |= (HAL_GPIO_ReadPin(SCCB_SDA_PORT,SCCB_SDA_PIN) & 0x01);	
		SCCB_SCL(0);
		SCCB_Delay( SCCB_DelayVaule );
	}
	
	SCCB_SDA_OUT();  // 切换回输出模式以发送ACK
	
	if ( ACK_Mode == 1 )				//	Ӧ���ź�
		SCCB_ACK();
	else
		SCCB_NoACK();		 	// ��Ӧ���ź�
	
	return IIC_Data; 
}


/*************************************************************************************************************************************
*	�� �� ��:	SCCB_WriteHandle
*
*	��ڲ���:	addr - Ҫ���в����ļĴ���(8λ��ַ)
*
*	�� �� ֵ:	SUCCESS - �����ɹ���ERROR	  - ����ʧ��
*					
*	��������:	��ָ���ļĴ���(8λ��ַ)ִ��д������OV2640�õ�
************************************************************************************************************************************/

uint8_t SCCB_WriteHandle (uint8_t addr)
{
	uint8_t status;		// ״̬��־λ

	SCCB_Start();	// ����IICͨ��
	if( SCCB_WriteByte(OV2640_DEVICE_ADDRESS) == ACK_OK ) //д����ָ��
	{
		if( SCCB_WriteByte((uint8_t)(addr)) != ACK_OK )
		{
			status = ERROR;	// ����ʧ��
		}			
	}
	status = SUCCESS;	// �����ɹ�
	return status;	
}

/*************************************************************************************************************************************
*	�� �� ��:	SCCB_WriteReg
*
*	��ڲ���:	addr - Ҫд��ļĴ���(8λ��ַ)��value - Ҫд�������
*					
*	�� �� ֵ:	SUCCESS - �����ɹ��� ERROR	  - ����ʧ��
*					
*	��������:	��ָ���ļĴ���(8λ��ַ)дһ�ֽ����ݣ�OV2640�õ�
************************************************************************************************************************************/

uint8_t SCCB_WriteReg (uint8_t addr,uint8_t value)
{
	uint8_t status;
	
	SCCB_Start(); //����IICͨѶ

	if( SCCB_WriteHandle(addr) == SUCCESS)	//д��Ҫ�����ļĴ���
	{
		if (SCCB_WriteByte(value) != ACK_OK) //д����
		{
			status = ERROR;						
		}
	}	
	SCCB_Stop(); // ֹͣͨѶ
	
	status = SUCCESS;	// д��ɹ�
	return status;
}
/*************************************************************************************************************************************
*	�� �� ��:	SCCB_ReadReg
*
*	��ڲ���:	addr - Ҫ��ȡ�ļĴ���(8λ��ַ)
*					
*	�� �� ֵ:	����������
*					
*	��������:	��ָ���ļĴ���(8λ��ַ)��ȡһ�ֽ����ݣ�OV2640�õ�
************************************************************************************************************************************/

uint8_t SCCB_ReadReg (uint8_t addr)
{
   uint8_t value = 0;

	SCCB_Start();		// ����IICͨ��

	if( SCCB_WriteHandle(addr) == SUCCESS) //д��Ҫ�����ļĴ���
	{
      SCCB_Stop();	// ֹͣIICͨ��
		SCCB_Start(); //��������IICͨѶ

		if (SCCB_WriteByte(OV2640_DEVICE_ADDRESS|0X01) == ACK_OK)	// ���Ͷ�����
		{	
			value = SCCB_ReadByte(0);	// �������һ������ʱ���� ��Ӧ���ź�
		}					
		SCCB_Stop();	// ֹͣIICͨ��

	}

	return value;	
}

/*************************************************************************************************************************************
*	�� �� ��:	SCCB_WriteHandle_16Bit
*
*	��ڲ���:	addr - Ҫ���в����ļĴ���(16λ��ַ)
*
*	�� �� ֵ:	SUCCESS - �����ɹ���ERROR - ����ʧ��
*					
*	��������:	��ָ���ļĴ���(16λ��ַ)ִ��д������OV5640�õ�
************************************************************************************************************************************/

uint8_t SCCB_WriteHandle_16Bit (uint16_t addr)
{
	uint8_t status;		// ״̬��־λ

	SCCB_Start();	// ����IICͨ��
	if( SCCB_WriteByte(OV5640_DEVICE_ADDRESS) == ACK_OK ) //д����ָ��
	{
		if( SCCB_WriteByte((uint8_t)(addr >> 8)) == ACK_OK ) //д��16λ��ַ
		{
			if( SCCB_WriteByte((uint8_t)(addr)) != ACK_OK )
			{
				status = ERROR;	// ����ʧ��
			}			
		}		
	}
	status = SUCCESS;	// �����ɹ�
	return status;	
}

/*************************************************************************************************************************************
*	�� �� ��:	SCCB_WriteReg_16Bit
*
*	��ڲ���:	addr - Ҫд��ļĴ���(16λ��ַ)  value - Ҫд�������
*					
*	�� �� ֵ:	SUCCESS - �����ɹ���ERROR	  - ����ʧ��
*					
*	��������:	��ָ���ļĴ���(16λ��ַ)дһ�ֽ����ݣ�OV5640�õ�
************************************************************************************************************************************/

uint8_t SCCB_WriteReg_16Bit(uint16_t addr,uint8_t value)
{
	uint8_t status;
	
	SCCB_Start(); //����IICͨѶ

	if( SCCB_WriteHandle_16Bit(addr) == SUCCESS)	//д��Ҫ�����ļĴ���
	{
		if (SCCB_WriteByte(value) != ACK_OK) //д����
		{
			status = ERROR;						
		}
	}	
	SCCB_Stop(); // ֹͣͨѶ
	
	status = SUCCESS;	// д��ɹ�
	return status;
}

/*************************************************************************************************************************************
*	�� �� ��:	SCCB_ReadReg_16Bit
*
*	��ڲ���:	addr - Ҫ��ȡ�ļĴ���(16λ��ַ)
*					
*	�� �� ֵ:	����������
*					
*	��������:	��ָ���ļĴ���(16λ��ַ)��ȡһ�ֽ����ݣ�OV5640�õ�
************************************************************************************************************************************/

uint8_t SCCB_ReadReg_16Bit (uint16_t addr)
{
   uint8_t value = 0;

	SCCB_Start();		// ����IICͨ��

	if( SCCB_WriteHandle_16Bit(addr) == SUCCESS) //д��Ҫ�����ļĴ���
	{
      SCCB_Stop();	// ֹͣIICͨ��
		SCCB_Start(); //��������IICͨѶ

		if (SCCB_WriteByte(OV5640_DEVICE_ADDRESS|0X01) == ACK_OK)	// ���Ͷ�����
		{	
			value = SCCB_ReadByte(0);	// �������һ������ʱ���� ��Ӧ���ź�
		}					
		SCCB_Stop();	// ֹͣIICͨ��

	}

	return value;	
}

/*************************************************************************************************************************************
*	�� �� ��:	SCCB_WriteBuffer_16Bit
*
*	��ڲ���:	addr - Ҫд��ļĴ���(16λ��ַ)  *pData - ������   size - Ҫ�������ݵĴ�С
*					
*	�� �� ֵ:	SUCCESS - �����ɹ���ERROR	  - ����ʧ��
*					
*	��������:	��ָ���ļĴ���(16λ��ַ)����д���ݣ�OV5640 д���Զ��Խ��̼�ʱ�õ�
************************************************************************************************************************************/

uint8_t SCCB_WriteBuffer_16Bit(uint16_t addr,uint8_t *pData, uint32_t size)
{
	uint8_t status;	
	uint32_t i;
	
	SCCB_Start(); //����IICͨѶ

	if( SCCB_WriteHandle_16Bit(addr) == SUCCESS)	//д��Ҫ�����ļĴ���
	{
		for(i=0;i<size;i++)
		{
			SCCB_WriteByte(*pData);//д����			
			pData++;
		}
	}	
	SCCB_Stop(); // ֹͣͨѶ
	
	status = SUCCESS;	// д��ɹ�
	return status;
}
/********************************************************************************************/
