#ifndef __DMA_PWM_H
#define __DMA_PWM_H

#include "main.h"
#include "dma.h"
#include "tim.h"


typedef struct
{
	//STM32������
	uint32_t Num_PWMtimeCCR;//����Ĵ�С
    uint32_t *Arr_PWMtimeCCR;//����
	TIM_HandleTypeDef* htim;//��ʱ��
	uint16_t TIM_CHANNEL;//��ʱ��ͨ��
	
	uint8_t  SendFlag;//���ͱ�־λ
} DMAPWM_Typedef;


void DMAPWM_Task();
extern DMAPWM_Typedef DMAPWM_Tim3_Channl2;

#endif

