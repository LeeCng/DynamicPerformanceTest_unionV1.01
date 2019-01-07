/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SD���ļ�ϵͳ����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 F103-�Ե� ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "./sdio/bsp_sdio_sdcard.h"
#include "./usart/bsp_usart.h"	
#include "./led/bsp_led.h"
#include "ff.h"
#include "./can/bsp_can.h"
#include "./adc/bsp_adc.h"
#include "./TimBase/bsp_TiMbase.h"
#include "./SysTick/bsp_SysTick.h"
#include "./HandlingData/HandlingData.h"
#include "math.h"
#include "./key/bsp_key.h"  
#include "./485/bsp_485.h"


#define pi 3.14159265
#define T_test  (1000*2*pi/fre[StartFlag-1])

#define fy_ID 0x00
#define ph_ID 0x02
/**
  ******************************************************************************
  *                              �������
  ******************************************************************************
  */
	
	//  ���Ա���
	uint8_t fre[27] = {1,1,2,3,5,6,10,15,20,25,30,35,40,45,50,55,60,65,70,80,90,100,110,120,130,140,150};    //  ɨ��Ƶ�� rad/s
	uint8_t NumT[27] = {0,4,5,5,6,8,10,10,10,10,10,20,20,20,20,20,20,20,30,30,30,30,30,30,30,30,30};				//  ���ڸ���
	
	/*      ��ǰֵΪ�������ڽǷ�ֵ����Ҫͨ������ת����Ϊ�ŷ������ڽǷ�ֵ*/
		float   A[2] = {130*0.5,130*1};			//   �������ڽ�0.5��
		uint16_t fy_mid = 1052;		// ���� ��λ
		
	//  ��������������
	uint8_t senddata[2] = {0};
	float TemData = 0;
	uint32_t TimeStart = 0;
	uint16_t Rx_Data_ceshi = 0;
	
  //  ������ƫ����������
	uint16_t fy_GoalPosl = 0;
	uint16_t ph_GoalPosl = 0;
	uint8_t WritePos[10] = {0xff,0xff,0xff,0x01,0x05,0xf3,0x86,0x00,0x00,0x80};
  //  ������ƫ���ŷ��ڷ���
  uint16_t fy_RxData = 0;
	uint16_t ph_RxData = 0;
	uint16_t Pos = 0;
	uint8_t running = 0;
	
	uint8_t  Rx_set[2]= {0};			//  ����������ת��
	uint8_t  Rx_real[2] = {0};		//  ������ʵ��ת��
	
	
uint8_t StartFlag = 0;		//�����Ƿ�ʼ��־λ	
uint8_t SaveFlag = 0;  		//�����ж��Ƿ���Ҫ��sd���������ݣ���ADC��ȡbuffer����ʱ��1 ��������պ�����
uint8_t SendFlag = 0;			// CAN����Flag
//uint8_t Pause = 1;			// �ж��Ƿ���ת��־λ
uint16_t k = 0;
uint8_t Key2_Flag = 0;			// ysm

// CAN 
__IO uint32_t flag = 0;		 //���ڱ�־�Ƿ���յ����ݣ����жϺ����и�ֵ
CanTxMsg TxMessage;			     //���ͻ�����
CanRxMsg RxMessage;				 //���ջ�����
uint16_t TxData =0x015e;                   // 8--2,{0}--{66,66} ysm
uint8_t mailbox_can = 0;
uint8_t timeflag = 0;

// ADC

extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];

// �ֲ����������ڱ���ת�������ĵ�ѹֵ 	 
//float ADC_ConvertedValueLocal[NOFCHANEL];        


// tim
volatile uint32_t time = 0; // ms ��ʱ���� 
uint32_t time_pause = 0; // ms ��ʱ���� 

uint8_t A_turn = 0;

uint32_t t1 = 0;
uint32_t t2 = 0;	


/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
		

	
	/* ��ʼ��LED */
	LED_GPIO_Config();	
	
		/*��ʼ��can,���жϽ���CAN���ݰ�*/
	CAN_Config();
	
		/*��ʼ������*/
	Key_GPIO_Config();
	
	/*   ��ʼ��TIM */
	BASIC_TIM_Init();
	
	/* ��ʼ�����Դ��ڣ�һ��Ϊ����1 */
	USART_Config();	
  printf("\r\n****** ����ϵͳ��̬���� ******\r\n");
	
	/*   ��ʼ�� ADC  */
	ADCx_Init();
  
	/*��ʼ��485 �����ж�ģʽ����*/
	RS485_Config();
	
	/* ��ʼ��SD�� */
	SDCard_Init();

	
	LED_RED;
	
	printf("\r\n ----ADC_TEST----\r\n");
	


	while(1)
	{
//			ADC_ConvertedValueLocal[0] =(float) ADC_ConvertedValue[0]/4096*3.3;
//			ADC_ConvertedValueLocal[1] =(float) ADC_ConvertedValue[1]/4096*3.3;
//			ADC_ConvertedValueLocal[2] =(float) ADC_ConvertedValue[2]/4096*3.3;
//			printf("\r\n CH1 value = %f V \r\n",ADC_ConvertedValueLocal[0]);
//			printf("\r\n CH2 value = %f V \r\n",ADC_ConvertedValueLocal[1]);
//			printf("\r\n CH3 value = %f V \r\n",ADC_ConvertedValueLocal[2]);
//		
//			printf("\r\n\r\n");
//			SysTick_Delay_Ms(500);
		
		if(Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON)				//		����Key1
		{	
			LED_GREEN;
			SysTick_Delay_Ms(1000);		//  ����������ǰ�ӳ�
			StartFlag++;
			TimeStart = time;			//  ��¼���Կ�ʼʱ��
		}

		if( StartFlag!=0 && StartFlag <= 28)		//  ��ʼ����
		{

			if(SendFlag == 1)
			{
				if((time-TimeStart)/T_test<=NumT[StartFlag-1])
				{
					LED_BLUE;
				//  ���㷢������
					fy_GoalPosl =(uint16_t)(fy_mid + A[A_turn]*sin(fre[StartFlag-1]*(float)(time-TimeStart)/1000));
//				ph_GoalPosl = fy_mid;
				//ͨ��485����
					t1 = time;
					fy_RxData = motor_ctl(fy_ID,fy_GoalPosl);
					t2 = time;
//				ph_RxData = motor_ctl(ph_ID,ph_GoalPosl);
					
//				printf(" \r\n fy_RxData = %d  \r\n",fy_RxData);
				

//				senddata[0] = 0;
//			  senddata[1] = 0;
//								
//				/*����Ҫ���͵ı���*/
//				CAN_SetMsg(&TxMessage,senddata);
//				/*�ѱ��Ĵ洢���������䣬����*/
//				CAN_Transmit(CANx, &TxMessage);
			
					SendFlag = 0;
				}else							//  �ж��Ƿ����Ԥ���������� ������ں���������Ϊ��λ
				{
					//���͸�����λ
					fy_GoalPosl = fy_mid;
					t1 = time;
					fy_RxData = motor_ctl(fy_ID,fy_GoalPosl);
					t2 = time;
//					ph_GoalPosl = A;
//					ph_RxData = motor_ctl(fy_ID,ph_GoalPosl);
//				printf(" \r\n fy_RxData = %d  \r\n",fy_RxData);
					LED_GREEN;
//					Pause = 1;
			/*  �ж��Ƿ�ִ����һ�β��ԣ����Խ�����1�룩 ����ʼʱ���Ϊ��ǰʱ��1*/
					if ((time-TimeStart-NumT[StartFlag-1]*T_test)>=1000)
					{
						TimeStart = time;
						StartFlag++;
					}				
				}		
				SendFlag = 0;
			}
			if (SaveFlag == 1)
			{
				Save2SD();
				k=0;
				SaveFlag=0;
			}
		}
					printf("\r\n  t1 = %d , t2 = %d \r\n",t1,t2);
		
	if(StartFlag>=28)
		{			
			LED_YELLOW;
			SysTick_Delay_Ms(2000);
			StartFlag = 1;
			A_turn++;
			printf("\r\n last A = %d \r\n",A_turn);
			if(A_turn>1)
			{
				Save2SD();
				while(1)
				{
				  SaveFlag=0;					
					LED_WHITE;
					k=0;
					StartFlag = 0;
				}
			}
		}
	}
}

	  

