/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   485����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:���� F103-�Ե� STM32  ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "./485/bsp_485.h"
#include <stdarg.h>

extern uint16_t Pos;
																		 


static void Delay(__IO u32 nCount); 


/// ����USART�����ж����ȼ�
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RS485_INT_IRQ; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 * ��������RS485_Config
 * ����  ��USART GPIO ����,����ģʽ����
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void RS485_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART clock */
	RCC_APB2PeriphClockCmd(RS485_USART_RX_GPIO_CLK|RS485_USART_TX_GPIO_CLK|RS485_RE_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(RS485_USART_CLK, ENABLE); 	
	
	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = RS485_USART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RS485_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  	
	// ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = RS485_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(RS485_USART_RX_GPIO_PORT, &GPIO_InitStructure);	
  
  /* 485�շ����ƹܽ� */
	GPIO_InitStructure.GPIO_Pin = RS485_RE_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	
  GPIO_Init(RS485_RE_GPIO_PORT, &GPIO_InitStructure);
	  
	/* USART ģʽ����*/
	USART_InitStructure.USART_BaudRate = RS485_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(RS485_USART, &USART_InitStructure); 
	/*ʹ��USART*/
  USART_Cmd(RS485_USART, ENABLE);
	
	/*�����ж����ȼ�*/
	NVIC_Configuration();
	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);
	
	/*����485оƬ�������ģʽ*/
	GPIO_ResetBits(RS485_RE_GPIO_PORT,RS485_RE_PIN);
}



/***************** ����һ���ַ�  **********************/
//ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ������š�
void RS485_SendByte(  uint8_t ch )
{
	/* ����һ���ֽ����ݵ�USART1 */
	USART_SendData(RS485_USART,ch);
		
	/* �ȴ�������� */
	while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TXE) == RESET);	
	
}
/*****************  ����ָ�����ȵ��ַ��� **********************/
void RS485_SendStr_length( uint8_t *str,uint32_t strlen )
{
	unsigned int k=0;

	RS485_TX_EN()	;//	ʹ�ܷ�������	
    do 
    {
        RS485_SendByte( *(str + k) );
        k++;
    } while(k < strlen);
		
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	RS485_RX_EN()	;//	ʹ�ܽ�������
}


/*****************  �����ַ��� **********************/
void RS485_SendString(  uint8_t *str)
{
	unsigned int k=0;
	
	RS485_TX_EN()	;//	ʹ�ܷ�������
	
    do 
    {
        RS485_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	RS485_RX_EN()	;//	ʹ�ܽ�������
}









//�жϻ��洮������
#define UART_BUFF_SIZE      1024
volatile    uint16_t uart_p = 0;
uint8_t     uart_buff[UART_BUFF_SIZE];

void bsp_RS485_IRQHandler(void)
{
    if(uart_p<UART_BUFF_SIZE)
    {
        if(USART_GetITStatus(RS485_USART, USART_IT_RXNE) != RESET)
        {
            uart_buff[uart_p] = USART_ReceiveData(RS485_USART);
            uart_p++;
						
						USART_ClearITPendingBit(RS485_USART, USART_IT_RXNE);
        }
    }
		else
		{
			USART_ClearITPendingBit(RS485_USART, USART_IT_RXNE);
//			clean_rebuff();       
		}
}



//��ȡ���յ������ݺͳ���
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}

//��ջ�����
void clean_rebuff(void) 
{

    uint16_t i=UART_BUFF_SIZE+1;
    uart_p = 0;
	while(i)
		uart_buff[--i]=0;

}




static void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}



uint16_t motor_ctl(uint8_t ID , uint16_t GoalPos1)
{

	uint16_t head = 0;
//	uint8_t step = 0;
	char *pbuf;
	uint8_t* a;
	uint16_t len,i;
	uint8_t ReadEcho[9]={0xff,0xff,0xff,0x00,0x04,0xf2,0x8c,0x02,0x7b};
	uint8_t WritePos[10]={0xff,0xff,0xff,0x01,0x05,0xf3,0x86,0x00,0x00,0x80};
	uint16_t sum1;	//////////////////////////////////////////////////
	WritePos[3] = ID;
	WritePos[7]=GoalPos1&0x00ff;//////////////////////////////////////////////////////
	WritePos[8]=(GoalPos1&0xff00)>>8;
  sum1=(ID+0x05+0xf3+0x86+WritePos[7]+WritePos[8])&0x00ff;
			

	WritePos[9]=0xffff-sum1;
	RS485_TX_EN();
			for(i=0;i<=9;i++)
			{
			RS485_SendByte(WritePos[i]);	 //????
			}
			Delay(0x1FFF);//????????????? 
			
			for(i=0;i<=8;i++)
			{
			RS485_SendByte(ReadEcho[i]);	 //????
			}
	    RS485_RX_EN();
			Delay(0x0fff);
//			RS485_SendStr_length(WritePos,sizeof(WritePos));//??????
				
//			RS485_SendStr_length(ReadEcho,sizeof(ReadEcho));//??????
				
			pbuf = get_rebuff(&len);
			a=(uint8_t*)pbuf; 
			
//			printf("\r\n  len = %d \r\n",len);
			
			if(len>=8 )
			{
				for(head = 0;head<len-7;head++)
				{
						if((a)[head] == 0xff && (a)[head+1] == 0xff && (a)[head+2] == 0xff && (a)[head+3] == 0x00)
						{
							 Pos=(a)[6]+(a)[7]*256;
//							 printf("\r\n v1.01 Pos = %d \r\n",Pos);
						}
				}								
//				printf("\r\n??????%d???\r\n",len);	
//				RS485_DEBUG_ARRAY((uint8_t*)pbuf,len);
				clean_rebuff();
			}


			return Pos;
}


