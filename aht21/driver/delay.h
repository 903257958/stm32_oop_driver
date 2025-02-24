#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"

#else
    #error delay.h: No processor defined!
#endif

#ifndef USE_FREERTOS
    #define USE_FREERTOS 0
#endif

#if !USE_FREERTOS
void delay_init(uint16_t sysclk_mhz);
#else
#include "timer.h"
void delay_init(TimerDev_t *timer);
#endif
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_s(uint32_t s);

#endif
