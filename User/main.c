/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SD卡文件系统例程
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F103-霸道 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
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
  *                              定义变量
  ******************************************************************************
  */
	
	//  测试变量
	uint8_t fre[27] = {1,1,2,3,5,6,10,15,20,25,30,35,40,45,50,55,60,65,70,80,90,100,110,120,130,140,150};    //  扫描频率 rad/s
	uint8_t NumT[27] = {0,4,5,5,6,8,10,10,10,10,10,20,20,20,20,20,20,20,30,30,30,30,30,30,30,30,30};				//  周期个数
	
	/*      当前值为发动机摆角幅值，需要通过测试转换成为伺服机构摆角幅值*/
		float   A[2] = {130*0.5,130*1};			//   发动机摆角0.5°
		uint16_t fy_mid = 1052;		// 俯仰 中位
		
	//  发动机发送命令
	uint8_t senddata[2] = {0};
	float TemData = 0;
	uint32_t TimeStart = 0;
	uint16_t Rx_Data_ceshi = 0;
	
  //  俯仰与偏航发送命令
	uint16_t fy_GoalPosl = 0;
	uint16_t ph_GoalPosl = 0;
	uint8_t WritePos[10] = {0xff,0xff,0xff,0x01,0x05,0xf3,0x86,0x00,0x00,0x80};
  //  俯仰与偏航伺服内反馈
  uint16_t fy_RxData = 0;
	uint16_t ph_RxData = 0;
	uint16_t Pos = 0;
	uint8_t running = 0;
	
	uint8_t  Rx_set[2]= {0};			//  发动机设置转速
	uint8_t  Rx_real[2] = {0};		//  发动机实际转速
	
	
uint8_t StartFlag = 0;		//程序是否开始标志位	
uint8_t SaveFlag = 0;  		//用于判断是否需要向sd卡储存数据，当ADC读取buffer存满时置1 ，储存接收后置零
uint8_t SendFlag = 0;			// CAN发送Flag
//uint8_t Pause = 1;			// 判断是否运转标志位
uint16_t k = 0;
uint8_t Key2_Flag = 0;			// ysm

// CAN 
__IO uint32_t flag = 0;		 //用于标志是否接收到数据，在中断函数中赋值
CanTxMsg TxMessage;			     //发送缓冲区
CanRxMsg RxMessage;				 //接收缓冲区
uint16_t TxData =0x015e;                   // 8--2,{0}--{66,66} ysm
uint8_t mailbox_can = 0;
uint8_t timeflag = 0;

// ADC

extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];

// 局部变量，用于保存转换计算后的电压值 	 
//float ADC_ConvertedValueLocal[NOFCHANEL];        


// tim
volatile uint32_t time = 0; // ms 计时变量 
uint32_t time_pause = 0; // ms 计时变量 

uint8_t A_turn = 0;

uint32_t t1 = 0;
uint32_t t2 = 0;	


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
		

	
	/* 初始化LED */
	LED_GPIO_Config();	
	
		/*初始化can,在中断接收CAN数据包*/
	CAN_Config();
	
		/*初始化按键*/
	Key_GPIO_Config();
	
	/*   初始化TIM */
	BASIC_TIM_Init();
	
	/* 初始化调试串口，一般为串口1 */
	USART_Config();	
  printf("\r\n****** 测试系统动态特性 ******\r\n");
	
	/*   初始化 ADC  */
	ADCx_Init();
  
	/*初始化485 采用中断模式接收*/
	RS485_Config();
	
	/* 初始化SD卡 */
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
		
		if(Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON)				//		按键Key1
		{	
			LED_GREEN;
			SysTick_Delay_Ms(1000);		//  发动机开机前延迟
			StartFlag++;
			TimeStart = time;			//  记录测试开始时间
		}

		if( StartFlag!=0 && StartFlag <= 28)		//  开始测试
		{

			if(SendFlag == 1)
			{
				if((time-TimeStart)/T_test<=NumT[StartFlag-1])
				{
					LED_BLUE;
				//  计算发送数据
					fy_GoalPosl =(uint16_t)(fy_mid + A[A_turn]*sin(fre[StartFlag-1]*(float)(time-TimeStart)/1000));
//				ph_GoalPosl = fy_mid;
				//通过485发送
					t1 = time;
					fy_RxData = motor_ctl(fy_ID,fy_GoalPosl);
					t2 = time;
//				ph_RxData = motor_ctl(ph_ID,ph_GoalPosl);
					
//				printf(" \r\n fy_RxData = %d  \r\n",fy_RxData);
				

//				senddata[0] = 0;
//			  senddata[1] = 0;
//								
//				/*设置要发送的报文*/
//				CAN_SetMsg(&TxMessage,senddata);
//				/*把报文存储到发送邮箱，发送*/
//				CAN_Transmit(CANx, &TxMessage);
			
					SendFlag = 0;
				}else							//  判断是否完成预定的周期数 完成周期后发送数据置为中位
				{
					//发送俯仰中位
					fy_GoalPosl = fy_mid;
					t1 = time;
					fy_RxData = motor_ctl(fy_ID,fy_GoalPosl);
					t2 = time;
//					ph_GoalPosl = A;
//					ph_RxData = motor_ctl(fy_ID,ph_GoalPosl);
//				printf(" \r\n fy_RxData = %d  \r\n",fy_RxData);
					LED_GREEN;
//					Pause = 1;
			/*  判断是否执行完一次测试（测试结束后1秒） 将开始时间存为当前时间1*/
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

	  

