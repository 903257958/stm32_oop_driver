#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef TIM_TypeDef*	TIMx;
	typedef GPIO_TypeDef*	PWM_GPIO_Port;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
	
	typedef TIM_TypeDef*	TIMx;
	typedef GPIO_TypeDef*	PWM_GPIO_Port;
#else
	#error pwm.h: No processor defined!
#endif

#ifndef pwm_log
	#define pwm_log(x) 
#endif

typedef struct {
	TIMx timx;						// 定时器外设
	uint8_t OCChannel;				// 输出比较通道
	uint16_t psc;					// PSC预分频器的值
	uint16_t arr;					// ARR自动重装器的值
	PWM_GPIO_Port port;				// PWM端口
	uint32_t pin;					// PWM引脚
}PWMInfo_t;

typedef struct PWMDev {
	PWMInfo_t info;
	bool initFlag;												// 初始化标志
	void (*set_psc)(struct PWMDev *pDev, uint16_t psc);			// PWM设置PSC的值
	void (*set_arr)(struct PWMDev *pDev, uint16_t arr);			// PWM设置ARR的值
	void (*set_compare)(struct PWMDev *pDev, uint16_t compare);	// PWM设置CCR的值
	int (*deinit)(struct PWMDev *pDev);							// 去初始化
}PWMDev_t;

int pwm_init(PWMDev_t *pDev);

#endif
