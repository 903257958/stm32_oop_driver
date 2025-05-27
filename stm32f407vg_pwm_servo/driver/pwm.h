#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	TimerPER_t;
	typedef GPIO_TypeDef*	PWMGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	TimerPER_t;
	typedef GPIO_TypeDef*	PWMGPIOPort_t;

#else
	#error pwm.h: No processor defined!
#endif

typedef struct {
	TimerPER_t timx;				// 定时器外设
	uint8_t oc_channel;				// 输出比较通道
	uint16_t psc;					// PSC预分频器的值
	uint16_t arr;					// ARR自动重装器的值
	PWMGPIOPort_t port;				// PWM端口
	uint32_t pin;					// PWM引脚
}PWMConfig_t;

typedef struct PWMDev {
	PWMConfig_t config;
	bool init_flag;												// 初始化标志
	void (*set_psc)(struct PWMDev *dev, uint16_t psc);			// PWM设置PSC的值
	void (*set_arr)(struct PWMDev *dev, uint16_t arr);			// PWM设置ARR的值
	void (*set_compare)(struct PWMDev *dev, uint16_t compare);	// PWM设置CCR的值
	int8_t (*deinit)(struct PWMDev *dev);						// 去初始化
}PWMDev_t;

int8_t pwm_init(PWMDev_t *dev);

#endif
