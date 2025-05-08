#include "DMA_PWM.h"
#include "WS2812.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


DMAPWM_Typedef DMAPWM_Tim3_Channl2={
.Num_PWMtimeCCR = NUM_LEDS * BITS_PER_LED+1,
.Arr_PWMtimeCCR = pwm_buffer,
	
.TIM_CHANNEL = TIM_CHANNEL_2,
.htim = &htim3,
};

//// PWM DMA 完成回调函数
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM3)
//	{
//		HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
//		DMAPWM_Tim3_Channl2.SendFlag=0;
//	}
//}

void DMAPWM_Control(DMAPWM_Typedef* object)
{
//	if(object->SendFlag==0)
//	{
	HAL_TIM_PWM_Start_DMA(object->htim, object->TIM_CHANNEL, (uint32_t *)object->Arr_PWMtimeCCR, sizeof(uint32_t)*object->Num_PWMtimeCCR / sizeof(uint32_t));
//		object->SendFlag=1;
//	}
	
	
}

void DMAPWM_Task()
{
	DMAPWM_Control(&DMAPWM_Tim3_Channl2);

}


