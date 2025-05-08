#include "GPIO_PWM.h"
#include "DWT_Delay.h"
#include "RGB.h"
#include "WS2812.h"
#include "math.h"

extern uint8_t RGB_buffer[NUM_LEDS * BITS_PER_LED][3];
// ���� GPIO �˿ں����ţ�ʾ����PC6��
#define WS2812_PORT     GPIOC
#define WS2812_PIN      6
#define WS2812_PIN_MASK (1 << WS2812_PIN)

// �������� GPIO �ߵ�ƽ����λ��
#define WS2812_HIGH()   (WS2812_PORT->BSRR = WS2812_PIN_MASK)
// �������� GPIO �͵�ƽ����λ��STM32F4 �� BSRR �� 16 λ��
#define WS2812_LOW()    (WS2812_PORT->BSRR = (WS2812_PIN_MASK << 16))


// ���� '1' �룺�ߵ�ƽ 700ns + �͵�ƽ 550ns
static inline void Send_1() {
    WS2812_HIGH();
    DWT_Delay_ns(550);
    WS2812_LOW();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//    DWT_Delay_ns(1);
}

// ���� '0' �룺�ߵ�ƽ 350ns + �͵�ƽ 900ns
static inline void Send_0() {
    WS2812_HIGH();

	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//    DWT_Delay_ns(10);
    WS2812_LOW();
    DWT_Delay_ns(550);
}

// ����һ���ֽڣ���λ�ȳ���
void WS2812_SendByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & (0x80 >> i)) Send_1();
        else Send_0();
    }
}
void WS2812_Reset() {
    WS2812_LOW();
    DWT_Delay_us(80);        // ���ֵ͵�ƽ ��50��s
}

void WS2812_SendRGB(uint8_t g, uint8_t r, uint8_t b) {
    WS2812_SendByte(g);  // WS2812 ʹ�� GRB ˳��
    WS2812_SendByte(r);
    WS2812_SendByte(b);
}

void WS2812_SendRGBArray(uint8_t rgb_array[][3], uint16_t led_count) {
    // ����ÿ�� LED
    for (uint16_t i = 0; i < led_count; i++) {
        // ���͵�ǰ LED �� GRB ���ݣ�ע�� WS2812 ʹ�� GRB ˳��
        WS2812_SendRGB(rgb_array[i][1], rgb_array[i][0], rgb_array[i][2]);
    }
    
    // ���͸�λ�źţ��������ݴ��䣩
    WS2812_Reset();
}
	
void GPIOPWM_Task()
{
	
   WS2812_SendRGBArray(RGB_buffer,NUM_LEDS);
   
}