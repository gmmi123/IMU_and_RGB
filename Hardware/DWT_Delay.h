#ifndef __DWT_Delay_H
#define __DWT_Delay_H

#include "main.h"

void DWT_Init(void);
void DWT_Delay_us(uint32_t us);
void DWT_Delay_ns(uint32_t ns);
#endif
