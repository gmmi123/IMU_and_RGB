#include "DWT_Delay.h"

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*((volatile uint32_t *)0xE0001000))
#define DWT_CYCCNT      (*((volatile uint32_t *)0xE0001004))
void DWT_Init(void)
{
    // 启用跟踪调试模块
    DEMCR |= (1 << 24);  // DEMCR_TRCENA_Msk
    
    // 重置周期计数器
    DWT_CYCCNT = 0;
    
    // 启用周期计数器
    DWT_CTRL |= (1 << 0);  // DWT_CTRL_CYCCNTENA_Msk
}
void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    // 168MHz下: 1us = 168个周期
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    
    while((DWT_CYCCNT - start) < cycles);
}

// 纳秒级延迟函数
void DWT_Delay_ns(uint32_t ns)
{
    uint32_t start = DWT_CYCCNT;
    // 计算需要的时钟周期数 (SystemCoreClock单位为Hz，1e9用于纳秒转换)
    // 使用浮点运算提高精度，然后四舍五入
    uint32_t cycles = (uint32_t)((float)ns * SystemCoreClock / 1e9f + 0.5f);
    
    // 处理计数器溢出情况
    while((DWT_CYCCNT - start) < cycles);
}

