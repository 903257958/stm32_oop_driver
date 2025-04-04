#ifndef __EXTI_H
#define __EXTI_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"

#else
	#error exti.h: No processor defined!
#endif

typedef struct {
	uint32_t exti_line;				// 中断通道
	void (*irq_callback)(void);		// 中断回调函数
}IRQHandler_t;

void irq_handler_register(uint32_t exti_line, void(*irq_callback)(void));
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#endif
