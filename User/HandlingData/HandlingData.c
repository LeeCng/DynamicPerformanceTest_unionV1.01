/**
  ******************************************************************************
  * @file    HandlingData.c
  * @author  licong
  * @version V1.0
  * @date    2083-09-10
  * @brief   ADC读取数据处理与储存
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
	
	#include "stm32f10x.h"
	#include "./usart/bsp_usart.h"	
	#include "HandlingData.h"
	#include "./adc/bsp_adc.h"
	
	uint8_t buffer[1012] = {0,0};				//临时储存数据 data_size*100+12
	char name[10] = {"0:/161.txt"};
	extern uint8_t A_turn;
	uint8_t last_turn = 0;

  extern uint16_t k ;			
	extern volatile uint32_t time ;
	extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];
	
	extern  uint8_t senddata[2];
	extern  uint8_t Rx_set[2];
	extern  uint16_t fy_RxData;
	extern  uint16_t ph_RxData;
	extern  uint16_t fy_GoalPosl;
	extern  uint16_t ph_GoalPosl;
	extern  uint8_t StartFlag;
	extern  uint8_t fre[27];
	
	uint8_t data_size = 10;
	
		// SDIO
	
FATFS fs;													/* FatFs文件系统对象 */
FIL fnew;													/* 文件对象 */
FIL *fn = &fnew;
FRESULT res_sd;                /* 文件操作结果 */
UINT fnum;            					  /* 文件成功读写数量 */
BYTE ReadBuffer[256]={0};        /* 读缓冲区 */
BYTE WriteBuffer[] = {0};              /* 写缓冲区*/
BYTE WriteFlag = 0;



void SDCard_Init(void)
{
	
//在外部SD卡挂载文件系统，文件系统挂载时会对SDIO设备初始化
	res_sd = f_mount(&fs,"0:",1);
	
/*----------------------- 格式化测试 ---------------------------*/  
	/* 如果没有文件系统就格式化创建创建文件系统 */
	if(res_sd == FR_NO_FILESYSTEM)
	{

    /* 格式化 */
		res_sd=f_mkfs("0:",0,0);							
		
		if(res_sd == FR_OK)
		{
      /* 格式化后，先取消挂载 */
			res_sd = f_mount(NULL,"0:",1);			
      /* 重新挂载	*/			
			res_sd = f_mount(&fs,"0:",1);
		}
	}

}
	
	
	//	中断调用，将时间与ADC读取量储存进buffer
	uint8_t logdata(void)
	{
		uint8_t nl = 2;
		buffer[0] = 0xff;
		buffer[1] = 0xff;
		
		//  记录时间
		buffer[data_size*k+nl]   = (uint8_t)((time&0xff000000)>>24);
		buffer[data_size*k+1+nl] = (uint8_t)((time&0x00ff0000)>>16);
		buffer[data_size*k+2+nl] = (uint8_t)((time&0x0000ff00)>>8);
		buffer[data_size*k+3+nl] = (uint8_t)(time&0x000000ff);
		
		//		记录俯仰指令  fy_input 
		buffer[data_size*k+4+nl] = (fy_GoalPosl&0xff00)>>8;
		buffer[data_size*k+5+nl] = fy_GoalPosl&0x00ff;
	
		//		记录俯仰反馈值 ADC   fy_XWY
		buffer[data_size*k+6+nl] = (ADC_ConvertedValue[0]&0xff00)>>8;
		buffer[data_size*k+7+nl] = ADC_ConvertedValue[0]&0x00ff;
		
		//		记录俯仰伺服反馈值 fy_nfk
		buffer[data_size*k+8+nl] = (fy_RxData&0xff00)>>8;
		buffer[data_size*k+9+nl] = fy_RxData&0x00ff;
//		printf("\r\n adc data =%d  \r\n",ADC_ConvertedValue[0]);

//		//		记录偏航指令  ph_input 
//		buffer[data_size*k+10+nl] = (ph_GoalPosl&0xff00)>>8;
//		buffer[data_size*k+11+nl] = ph_GoalPosl&0x00ff;
//	
//		//		记录偏航反馈值 ADC   ph_XWY
//		buffer[data_size*k+12+nl] = (ADC_ConvertedValue[2]&0xff00)>>8;
//		buffer[data_size*k+13+nl] = ADC_ConvertedValue[2]&0x00ff;
//		
//		//		记录偏航伺服反馈值 ph_nfk
//		buffer[data_size*k+14+nl] = (ph_RxData&0xff00)>>8;
//		buffer[data_size*k+15+nl] = ph_RxData&0x00ff;



		
		
		k++;
		
		if (k>=100-1)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	
void Save2SD(void)
{

//	if(A_turn!=last_turn)
//	{
//		last_turn = A_turn;
//		name[5]= (char)('1'+A_turn);
////		printf("\r\n  name \r\n");
//	}
	if(A_turn == 0)
	{
		res_sd = f_open(fn,"0:/181.txt",FA_OPEN_ALWAYS | FA_WRITE  );
	}else
	{
		res_sd = f_open(fn,"0:/182.txt",FA_OPEN_ALWAYS | FA_WRITE  );
	}
	f_lseek(fn,f_size(fn));
	if(FR_OK == f_write(fn,buffer,sizeof(buffer),&fnum))
	{	
		k = 0;
	}else
	{
		LED_RED;
	}
	f_close(fn);
		
}

