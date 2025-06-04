#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>
#include <stdbool.h>

#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
    defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL) 
	#include "stm32f10x.h"
	
#elif defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || \
    defined(STM32F401xx) || defined(STM32F410xx) || defined(STM32F411xE) || defined(STM32F412xG) || \
    defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
	#include "stm32f4xx.h"
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD) || defined (GD32F10X_XD) || defined (GD32F10X_CL)
    #include "gd32f10x.h"

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
