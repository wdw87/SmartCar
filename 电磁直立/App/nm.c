#include "common.h"
#include "nm.h"



u8 TxBuffer[256] = {0};
u8 TxCounter=0;
u8 count=0;
static u8 RxBuffer[50];
static u8 RxState = 0;

static void Uart1_Put_Char(unsigned char DataToSend);
static void Recive_Analyse(uint8_t *data_buf,uint8_t num);
void Send_Data(uint8_t *dataToSend , uint8_t length);

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


void Usart1_Init(void)
{
	GPIO_InitTypeDef 		GPIO_InitStructure;
	USART_InitTypeDef 		USART_InitStructure;
	USART_ClockInitTypeDef 	USART_ClockInitStruct;
	
	//��λ����1
 	USART_DeInit(USART1); 
	
	//USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;					//�����������	
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;			//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate =115200;						//������115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;				//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	//����USART1ʱ��
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  		//ʱ�ӵ͵�ƽ�
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  			//SLCK������ʱ������ļ���->�͵�ƽ
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  			//ʱ�ӵڶ������ؽ������ݲ���
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; 	//���һλ���ݵ�ʱ�����岻��SCLK���

    USART_Init(USART1, &USART_InitStructure); 
	USART_ClockInit(USART1, &USART_ClockInitStruct);				
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);					//ʹ�ܽ����ж�
    USART_Cmd(USART1, ENABLE);                    					//ʹ�ܴ��� 
// 	
// 	DMA_DeInit(DMA1_Channel4);   									//��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
// 	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1->DR;  		//DMA����ADC����ַ
// 	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&TxBuffer;  	//DMA�ڴ����ַ
// 	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  			//���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
// 	DMA_InitStructure.DMA_BufferSize = 500;  						//DMAͨ����DMA����Ĵ�С
// 	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�Ĵ�������
// 	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  		//�ڴ��ַ�Ĵ�������
// 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
// 	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
// 	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  					//��������������ģʽ
// 	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 			//DMAͨ�� xӵ�������ȼ� 
// 	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  					//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
// 	
// 	DMA_Init(DMA1_Channel4, &DMA_InitStructure);  					//����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
// //	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);					//����DMA��������ж�
// 	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
}

//�ض���c�⺯��printf��USART1
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ�USART1 */
		USART_SendData(USART1, (uint8_t) ch);

		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

//�ض���c�⺯��scanf��USART1
int fgetc(FILE *f)
{
		/* �ȴ�����1�������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}

static void Uart1_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}

void Uart1_Put_String(unsigned char *Str)
{
	//�ж�Strָ��������Ƿ���Ч.
	while(*Str)
	{
	//�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
	if(*Str=='\r')Uart1_Put_Char(0x0d);
		else if(*Str=='\n')Uart1_Put_Char(0x0a);
			else Uart1_Put_Char(*Str);
	//ָ��++ ָ����һ���ֽ�.
	Str++;
	}
}

void DMA_Send(unsigned char *DataToSend , u8 data_num)
{

}


void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num)
{
	uint8_t i;
	for(i = 0;i < data_num;i++)
		TxBuffer[count++] = *(DataToSend+i);
//�жϷ�ʽ
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}

void Send_Check(uint8_t head, uint8_t check_sum)
{
	uint8_t i, sum = 0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;
	
	Send_Data(data_to_send, 7);
}

static void Recive_Analyse(uint8_t *data_buf,uint8_t num)
{
	uint8_t i, sum = 0;
	
	for(i = 0;i < (num - 1);i++)
		sum += *(data_buf + i);
	if(!(sum == *(data_buf + num - 1)))		return;		//�ж�sum
	if(!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))		return;		//�ж�֡ͷ

/////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			f.ACC_CALIBRATED = 1;
		if(*(data_buf+4)==0X02)
			f.GYRO_CALIBRATED = 1;
		if(*(data_buf+4)==0X03)
		{
			f.ACC_CALIBRATED = 1;		
			f.GYRO_CALIBRATED = 1;			
		}
		
		if(*(data_buf+4)==0X04)
		{
			f.MAG_CALIBRATED = 1;		
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
 			dt_f.Send_PID1 = 1;
 			dt_f.Send_PID2 = 1;
 			dt_f.Send_PID3 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
	}

	if(*(data_buf+2)==0X10)								//PID1
	{
		PID[ANGLE_ROL].Kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5)) * 0.01;
		PID[ANGLE_ROL].Ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7)) * 0.01;
		PID[ANGLE_ROL].Kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9)) * 0.001;
		PID[ANGLE_PIT].Kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11)) * 0.01;
		PID[ANGLE_PIT].Ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13)) * 0.01;
		PID[ANGLE_PIT].Kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15)) * 0.001;
		PID[ANGLE_YAW].Kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17)) * 0.01;
		PID[ANGLE_YAW].Ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19)) * 0.01;
		PID[ANGLE_YAW].Kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21)) * 0.001;
		Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
		PID[RATE_ROL].Kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5)) * 0.01;
		PID[RATE_ROL].Ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7)) * 0.01;
		PID[RATE_ROL].Kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9)) * 0.001;
		PID[RATE_PIT].Kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11)) * 0.01;
		PID[RATE_PIT].Ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13)) * 0.01;
		PID[RATE_PIT].Kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15)) * 0.001;
		PID[RATE_YAW].Kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17)) * 0.01;
		PID[RATE_YAW].Ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19)) * 0.01;
		PID[RATE_YAW].Kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21)) * 0.001;
		Send_Check(*(data_buf+2),sum);
	}
// 	if(*(data_buf+2)==0X12)								//PID3
// 	{
// 		Send_Check(sum);
// 		param.SAVE_PID();
// 	}
// 	if(*(data_buf+2)==0X16)								//OFFSET
// 	{

// 	}

// /////////////////////////////////////////////////////////////////////////////////////////////////
// 	if(*(data_buf+2)==0x18)					
// 	{

// 	}
}


/*
void USART1_IRQHandler(void)
{
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//??!????if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)???
    {
        USART_ReceiveData(USART1);
    }
	
	//�����ж�
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++];    //дDR����жϱ�־ 
			
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�ж�
			//count = 0;
			//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	//�����ж� (���ռĴ����ǿ�) 
	if(USART1->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{
		u8 com_data = USART1->DR;
		static u8 _data_len = 0,_data_cnt = 0;
		if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			
			Recive_Analyse(RxBuffer,_data_cnt+5);
		//	dt.Data_Receive_Anl(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	}
}
*/
void Send_Data(uint8_t *dataToSend , uint8_t length)
{
	Uart1_Put_Buf(dataToSend , length);
}

void Send_Status(void)
{
	u8  i,_cnt=0,sum = 0;
	vs16 _temp  =0;
	vs32 _temp2 = Sonar.Alt*100;	
	vs8  _temp1  = 0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=12;
	
	//��̬
	_temp = (int)(angle.roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(-angle.pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle.yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//�߶�
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	//����ģʽ
	data_to_send[_cnt++]=BYTE0(_temp1);		
	//����״̬
	_temp1 = f.ARMED;	
	data_to_send[_cnt++]=BYTE0(_temp1);	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Send_Senser(void )
{
	u8 i, sum = 0,  _cnt=0;
	vs16 _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=18;

	data_to_send[_cnt++]=BYTE1(Mpu6050.Acc_Fifter.X);
	data_to_send[_cnt++]=BYTE0(Mpu6050.Acc_Fifter.X);
	data_to_send[_cnt++]=BYTE1(Mpu6050.Acc_Fifter.Y);
	data_to_send[_cnt++]=BYTE0(Mpu6050.Acc_Fifter.Y);
	data_to_send[_cnt++]=BYTE1(Mpu6050.Acc_Fifter.Z);
	data_to_send[_cnt++]=BYTE0(Mpu6050.Acc_Fifter.Z);
	data_to_send[_cnt++]=BYTE1(Mpu6050.Gyro.X);
	data_to_send[_cnt++]=BYTE0(Mpu6050.Gyro.X);
	data_to_send[_cnt++]=BYTE1(Mpu6050.Gyro.Y);
	data_to_send[_cnt++]=BYTE0(Mpu6050.Gyro.Y);
	data_to_send[_cnt++]=BYTE1(Mpu6050.Gyro.Z);
	data_to_send[_cnt++]=BYTE0(Mpu6050.Gyro.Z);
	data_to_send[_cnt++]=BYTE1(HMC5883L.Fifter.X);
	data_to_send[_cnt++]=BYTE0(HMC5883L.Fifter.X);
	data_to_send[_cnt++]=BYTE1(HMC5883L.Fifter.Y);
	data_to_send[_cnt++]=BYTE0(HMC5883L.Fifter.Y);
	data_to_send[_cnt++]=BYTE1(HMC5883L.Fifter.Z);
	data_to_send[_cnt++]=BYTE0(HMC5883L.Fifter.Z);
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
}
void Send_PID1(void)
{
	u8 i, sum = 0,  _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=18;

	_temp = PID[ANGLE_ROL].Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[ANGLE_ROL].Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[ANGLE_ROL].Kd * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[ANGLE_PIT].Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[ANGLE_PIT].Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[ANGLE_PIT].Kd * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[ANGLE_YAW].Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[ANGLE_YAW].Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[ANGLE_YAW].Kd * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
}

void Send_PID2(void)
{
	u8 i, sum = 0,  _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=18;

	_temp = PID[RATE_ROL].Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[RATE_ROL].Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[RATE_ROL].Kd * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[RATE_PIT].Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[RATE_PIT].Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[RATE_PIT].Kd * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[RATE_YAW].Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[RATE_YAW].Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PID[RATE_YAW].Kd * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
}

void Send_RCData(void)
{
	u8 i, sum = 0,_cnt=0;
	uint16_t _temp=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=20;
	data_to_send[_cnt++]=BYTE1(Rc.Data[THROTTLE]);
	data_to_send[_cnt++]=BYTE0(Rc.Data[THROTTLE]);
	
	data_to_send[_cnt++]=BYTE1(Rc.Data[YAW]);
	data_to_send[_cnt++]=BYTE0(Rc.Data[YAW]);
	data_to_send[_cnt++]=BYTE1(Rc.Data[ROLL]);
	data_to_send[_cnt++]=BYTE0(Rc.Data[ROLL]);
	data_to_send[_cnt++]=BYTE1(Rc.Data[PITCH]);
	data_to_send[_cnt++]=BYTE0(Rc.Data[PITCH]);
	data_to_send[_cnt++]=BYTE1(Rc.Data[AUX1]);
	data_to_send[_cnt++]=BYTE0(Rc.Data[AUX1]);
	data_to_send[_cnt++]=BYTE1(Rc.Data[AUX2]);
	data_to_send[_cnt++]=BYTE0(Rc.Data[AUX2]);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Send_MotoPWM(void)
{
	u8 i, sum =0, _cnt=0;
	uint16_t _temp = 0, Moto_PWM[4];
	
	for(i=0;i<4;i++)
		Moto_PWM[i] = Motor[i] - 1000;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=16;
	data_to_send[_cnt++]=BYTE1(Moto_PWM[0]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[0]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[1]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[1]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[2]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[2]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[3]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[3]);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}



void Send_USER_DATA(void)
{
	uint8_t i=0, sum = 0, _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=14;


	//����ֵ
	data_to_send[_cnt++]=BYTE3(Sonar.Alt);
	data_to_send[_cnt++]=BYTE2(Sonar.Alt);
	data_to_send[_cnt++]=BYTE1(Sonar.Alt);
	data_to_send[_cnt++]=BYTE0(Sonar.Alt);
	
	//ʵ��ֵ
	///*data_to_send[_cnt++]=BYTE3(angle.yaw);
	//data_to_send[_cnt++]=BYTE2(angle.yaw);
	data_to_send[_cnt++]=BYTE1(Rc.Data[AUX1]);
	data_to_send[_cnt++]=BYTE0(Rc.Data[AUX1]);
	
	data_to_send[_cnt++]=BYTE1(Rc.Data[AUX2]);
	data_to_send[_cnt++]=BYTE0(Rc.Data[AUX2]);
	
 
	data_to_send[_cnt++]=BYTE3(PID[Alt].Output);
	data_to_send[_cnt++]=BYTE2(PID[Alt].Output);
	data_to_send[_cnt++]=BYTE1(PID[Alt].Output);
	data_to_send[_cnt++]=BYTE0(PID[Alt].Output);
	
	//�⻷P
 	data_to_send[_cnt++]=BYTE1(AD.Battery_Voltage);
 	data_to_send[_cnt++]=BYTE0(AD.Battery_Voltage);
/*	data_to_send[_cnt++]=BYTE1(PID[ANGLE_YAW].Proportion);
	data_to_send[_cnt++]=BYTE0(PID[ANGLE_YAW].Proportion);
	//�ڻ�P
	data_to_send[_cnt++]=BYTE3(PID[RATE_YAW].Proportion);
 	data_to_send[_cnt++]=BYTE2(PID[RATE_YAW].Proportion);
	data_to_send[_cnt++]=BYTE1(PID[RATE_YAW].Proportion);
	data_to_send[_cnt++]=BYTE0(PID[RATE_YAW].Proportion);
	//�ڻ�I
	data_to_send[_cnt++]=BYTE3(PID[RATE_YAW].Integral);
 	data_to_send[_cnt++]=BYTE2(PID[RATE_YAW].Integral);
	data_to_send[_cnt++]=BYTE1(PID[RATE_YAW].Integral);
	data_to_send[_cnt++]=BYTE0(PID[RATE_YAW].Integral);
	//�ڻ�D
	data_to_send[_cnt++]=BYTE3(PID[RATE_YAW].Derivative);
 	data_to_send[_cnt++]=BYTE2(PID[RATE_YAW].Derivative);
	data_to_send[_cnt++]=BYTE1(PID[RATE_YAW].Derivative);
	data_to_send[_cnt++]=BYTE0(PID[RATE_YAW].Derivative);
    
*/
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Data_Exchange(void)
{
	static u8 cnt = 0;
	
	switch(cnt)
	{
		case 1: 
			dt_f.Send_RCData = 1;
			break;
		case 2:
			dt_f.Send_MotoPwm = 1;
			break;
		case 30:
			cnt = 0;
			break;
		default:
			if(cnt%3)
				dt_f.Send_Senser = 1;	
			else
				dt_f.Send_Status = 1;
						
	}
	cnt++;
	
	if(dt_f.Send_Status){
		dt_f.Send_Status = 0;
		Send_Status();
	}	
	if(dt_f.Send_Senser){
		dt_f.Send_Senser = 0;
		Send_Senser();
	}	
	if(dt_f.Send_RCData){
		dt_f.Send_RCData = 0;
		Send_RCData();
	}		
	if(dt_f.Send_MotoPwm){
		dt_f.Send_MotoPwm = 0;
		Send_MotoPWM();
	}	
	if(dt_f.Send_PID1){
		dt_f.Send_PID1 = 0;
		Send_PID1();
	}	
	if(dt_f.Send_PID2){
		dt_f.Send_PID2 = 0;
		Send_PID2();
	}	
 	Send_USER_DATA();
// 	Send_Status();
}
