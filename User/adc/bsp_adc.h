#ifndef __BSP_ADC_H
#define	__BSP_ADC_H


#include "stm32f10x.h"

// ADC GPIO�궨��
// ע�⣺����ADC�ɼ���IO����û�и��ã�����ɼ���ѹ����Ӱ��
#define    ADC_GPIO_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define    ADC_GPIO_CLK                  RCC_APB2Periph_GPIOC  
#define    ADC_PORT                      GPIOC
#define    NOFCHANEL										 3


#define    ADC_PIN1                      GPIO_Pin_1
#define    ADC_CHANNEL1                  ADC_Channel_11

#define    ADC_PIN2                      GPIO_Pin_2
#define    ADC_CHANNEL2                  ADC_Channel_12

#define    ADC_PIN3                      GPIO_Pin_3
#define    ADC_CHANNEL3                  ADC_Channel_13



// ADC ���ѡ��
// ������ ADC1/2�����ʹ��ADC3���ж���ص�Ҫ�ĳ�ADC3��
#define    ADC_APBxClock_FUN             RCC_APB2PeriphClockCmd
#define    ADC_x                         ADC1
#define    ADC_CLK                       RCC_APB2Periph_ADC1

// ADC ͨ���궨��


#define    ADC_DMA_CLK                   RCC_AHBPeriph_DMA1
#define    ADC_DMA_CHANNEL               DMA1_Channel1

void ADCx_Init(void);

#endif /* __BSP_ADC_H */


