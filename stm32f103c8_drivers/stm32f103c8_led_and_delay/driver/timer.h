#ifndef TIMER_DRV_H
#define TIMER_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	timer_periph_t;

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	timer_periph_t;

#else
	#error timer.h: No processor defined!
#endif

typedef struct {
	timer_periph_t timx;		// 定时器外设
	uint16_t psc;				// PSC预分频器
	uint16_t arr;				// ARR自动重装器
	void (*irq_callback)(void);	// 定时中断回调函数
} timer_config_t;

typedef struct timer_dev {
	timer_config_t config;
	bool init_flag;											// 初始化标志
	void *priv_data;										// 私有数据指针
	int (*delay_us)(struct timer_dev *dev, uint32_t us);	// 微秒级延时，需要正确配置PSC寄存器
	int (*delay_ms)(struct timer_dev *dev, uint32_t ms);	// 毫秒级延时，需要正确配置PSC寄存器
	int8_t (*deinit)(struct timer_dev *dev);				// 去初始化
} timer_dev_t;

int8_t timer_init(timer_dev_t *dev);

#endif
