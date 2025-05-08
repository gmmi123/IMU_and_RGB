#include "WS2812.h"

uint32_t pwm_buffer[NUM_LEDS * BITS_PER_LED+1]; // 每个比特占一个uint32_t

 // 每个比特占一个uint32_t
extern uint8_t RGB_buffer[NUM_LEDS * BITS_PER_LED][3];
WS2812_Typedef WS2812={
.pwm_buffer = pwm_buffer,
.pwm_buffer_num = NUM_LEDS * BITS_PER_LED+1,
.RGB_buffer = RGB_buffer,
.RGB_Num = NUM_LEDS,
.mod =0,
};
//位操作填充
void fill_buffer_fast(uint32_t *pwm_buf, const uint8_t (*led_colors)[3], uint16_t num_eds) {
    for (uint16_t led = 0; led < num_eds; led++) {
        uint8_t r = led_colors[led][0]; // WS2812 是 GRB 顺序
        uint8_t g = led_colors[led][1];
        uint8_t b = led_colors[led][2];
        
        uint32_t *buf_ptr = &pwm_buf[led * BITS_PER_LED];
        
        // 处理 G（绿色，8位）
        for (uint8_t i = 0; i < 8; i++) {
            buf_ptr[i] = ((g << i) & 0x80) ? 59 : 29;
        }
        
        // 处理 R（红色，8位）
        for (uint8_t i = 0; i < 8; i++) {
            buf_ptr[i + 8] = ((r << i) & 0x80) ? 59 : 29;
        }
        
        // 处理 B（蓝色，8位）
        for (uint8_t i = 0; i < 8; i++) {
            buf_ptr[i + 16] = ((b << i) & 0x80) ? 59 : 29;
        }
    }
}

//查表法

#define BITS_PER_BYTE 8
// 预生成 256 种颜色的 8 位占空比序列
uint32_t lut[256][BITS_PER_BYTE];
void generate_lut() {
    for (uint16_t color = 0; color < 256; color++) {
        for (uint8_t bit = 0; bit < BITS_PER_BYTE; bit++) {
            lut[color][bit] = (color & (1 << (7 - bit))) ? 59 : 29;
        }
    }
}
void fill_buffer_lut(uint32_t *pwm_buf,uint8_t (*led_colors)[3], uint16_t num_eds) {
    for (uint16_t led = 0; led < num_eds; led++) {
        uint8_t r = led_colors[led][0]; // WS2812 是 GRB 顺序
        uint8_t g = led_colors[led][1];
        uint8_t b = led_colors[led][2];
        
        uint32_t *buf_ptr = &pwm_buf[led * BITS_PER_LED];
        
        // 复制 G 的 8 位占空比
        memcpy(&buf_ptr[0], lut[g], BITS_PER_BYTE * sizeof(uint32_t));
        
        // 复制 R 的 8 位占空比
        memcpy(&buf_ptr[8], lut[r], BITS_PER_BYTE * sizeof(uint32_t));
        
        // 复制 B 的 8 位占空比
        memcpy(&buf_ptr[16], lut[b], BITS_PER_BYTE * sizeof(uint32_t));
    }
}

void WS2812_control(WS2812_Typedef* object)
{
	//将RGB信息转为PWM信息
	if(object->mod==0)//位操作
	{
		fill_buffer_fast(object->pwm_buffer,object->RGB_buffer,object->RGB_Num);
	}
	else
	{
		static uint8_t initflg;
		if(initflg==0)
		{
			generate_lut();
			initflg=1;
		}
		fill_buffer_lut(object->pwm_buffer,object->RGB_buffer,object->RGB_Num);
	}
}


void WS2812_Task()
{
	WS2812_control(&WS2812);
}


