#include "DWT_Delay.h"

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*((volatile uint32_t *)0xE0001000))
#define DWT_CYCCNT      (*((volatile uint32_t *)0xE0001004))
void DWT_Init(void)
{
    // ���ø��ٵ���ģ��
    DEMCR |= (1 << 24);  // DEMCR_TRCENA_Msk
    
    // �������ڼ�����
    DWT_CYCCNT = 0;
    
    // �������ڼ�����
    DWT_CTRL |= (1 << 0);  // DWT_CTRL_CYCCNTENA_Msk
}
void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    // 168MHz��: 1us = 168������
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    
    while((DWT_CYCCNT - start) < cycles);
}

// ���뼶�ӳٺ���
void DWT_Delay_ns(uint32_t ns)
{
    uint32_t start = DWT_CYCCNT;
    // ������Ҫ��ʱ�������� (SystemCoreClock��λΪHz��1e9��������ת��)
    // ʹ�ø���������߾��ȣ�Ȼ����������
    uint32_t cycles = (uint32_t)((float)ns * SystemCoreClock / 1e9f + 0.5f);
    
    // ���������������
    while((DWT_CYCCNT - start) < cycles);
}

