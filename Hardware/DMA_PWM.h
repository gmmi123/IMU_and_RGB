#ifndef __DMA_PWM_H
#define __DMA_PWM_H

#include "main.h"
#include "dma.h"
#include "tim.h"


typedef struct
{
	//STM32特性区
	uint32_t Num_PWMtimeCCR;//数组的大小
    uint32_t *Arr_PWMtimeCCR;//数组
	TIM_HandleTypeDef* htim;//定时器
	uint16_t TIM_CHANNEL;//定时器通道
	
	uint8_t  SendFlag;//发送标志位
} DMAPWM_Typedef;


void DMAPWM_Task();
extern DMAPWM_Typedef DMAPWM_Tim3_Channl2;

#endif

