#ifndef __WS2812_H
#define __WS2812_H

#define BITS_PER_LED 24		//每个RGB需要传输的数据位数
#include "RGB.h"
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "string.h"
typedef struct
{
	uint32_t *pwm_buffer;		//转换后的pwm数据缓存区
	uint32_t pwm_buffer_num;
	
	uint8_t (*RGB_buffer)[3];	//RGB数据缓存区

	uint32_t RGB_Num;
	
	uint16_t mod;//0为位操作转换PWM，1为查表法
}WS2812_Typedef;

extern WS2812_Typedef WS2812;
extern uint32_t pwm_buffer[NUM_LEDS * BITS_PER_LED+1];

void WS2812_Task(void);

#endif
