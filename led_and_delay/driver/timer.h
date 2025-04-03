#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef* TimerPER_t;

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef* TimerPER_t;

#else
	#error timer.h: No processor defined!
#endif

typedef struct {
	TimerPER_t timx;				// 定时器外设
	uint16_t psc;					// PSC预分频器
	uint16_t arr;					// ARR自动重装器
	void (*irq_callback)(void);		// 定时中断回调函数
}TimerConfig_t;

typedef struct TimerDev {
	TimerConfig_t config;
	bool init_flag;											// 初始化标志
	void *priv_data;										// 私有数据指针
	int (*delay_us)(struct TimerDev *dev, uint32_t us);	// 微秒级延时，需要正确配置PSC寄存器
	int (*delay_ms)(struct TimerDev *dev, uint32_t ms);	// 毫秒级延时，需要正确配置PSC寄存器
	int (*deinit)(struct TimerDev *dev);					// 去初始化
}TimerDev_t;

int timer_init(TimerDev_t *dev);

#endif
