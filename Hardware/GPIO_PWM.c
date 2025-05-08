#include "GPIO_PWM.h"
#include "DWT_Delay.h"
#include "RGB.h"
#include "WS2812.h"
#include "math.h"

extern uint8_t RGB_buffer[NUM_LEDS * BITS_PER_LED][3];
// 定义 GPIO 端口和引脚（示例：PC6）
#define WS2812_PORT     GPIOC
#define WS2812_PIN      6
#define WS2812_PIN_MASK (1 << WS2812_PIN)

// 快速设置 GPIO 高电平（置位）
#define WS2812_HIGH()   (WS2812_PORT->BSRR = WS2812_PIN_MASK)
// 快速设置 GPIO 低电平（复位，STM32F4 用 BSRR 高 16 位）
#define WS2812_LOW()    (WS2812_PORT->BSRR = (WS2812_PIN_MASK << 16))


// 发送 '1' 码：高电平 700ns + 低电平 550ns
static inline void Send_1() {
    WS2812_HIGH();
    DWT_Delay_ns(550);
    WS2812_LOW();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//    DWT_Delay_ns(1);
}

// 发送 '0' 码：高电平 350ns + 低电平 900ns
static inline void Send_0() {
    WS2812_HIGH();

	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//    DWT_Delay_ns(10);
    WS2812_LOW();
    DWT_Delay_ns(550);
}

// 发送一个字节（高位先出）
void WS2812_SendByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & (0x80 >> i)) Send_1();
        else Send_0();
    }
}
void WS2812_Reset() {
    WS2812_LOW();
    DWT_Delay_us(80);        // 保持低电平 ≥50μs
}

void WS2812_SendRGB(uint8_t g, uint8_t r, uint8_t b) {
    WS2812_SendByte(g);  // WS2812 使用 GRB 顺序！
    WS2812_SendByte(r);
    WS2812_SendByte(b);
}

void WS2812_SendRGBArray(uint8_t rgb_array[][3], uint16_t led_count) {
    // 遍历每个 LED
    for (uint16_t i = 0; i < led_count; i++) {
        // 发送当前 LED 的 GRB 数据（注意 WS2812 使用 GRB 顺序）
        WS2812_SendRGB(rgb_array[i][1], rgb_array[i][0], rgb_array[i][2]);
    }
    
    // 发送复位信号（结束数据传输）
    WS2812_Reset();
}
	
void GPIOPWM_Task()
{
	
   WS2812_SendRGBArray(RGB_buffer,NUM_LEDS);
   
}