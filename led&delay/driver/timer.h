#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef TIM_TypeDef* TIMx;

#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
	
	typedef TIM_TypeDef* TIMx;

#else
	#error timer.h: No processor defined!
#endif

#ifndef timer_log
	#define timer_log(x) 
#endif

typedef struct {
	TIMx timx;						// 定时器外设
	uint16_t psc;					// PSC预分频器
	uint16_t arr;					// ARR自动重装器
	void (*irq_callback)(void);		// 定时中断回调函数
}TimerInfo_t;

typedef struct TimerDev {
	TimerInfo_t info;
	bool initFlag;											// 初始化标志
	void *pPrivData;										// 私有数据指针
	int (*delay_us)(struct TimerDev *pDev, uint32_t us);	// 微秒级延时，需要正确配置PSC寄存器
	int (*delay_ms)(struct TimerDev *pDev, uint32_t ms);	// 毫秒级延时，需要正确配置PSC寄存器
	int (*deinit)(struct TimerDev *pDev);					// 去初始化
}TimerDev_t;

int timer_init(TimerDev_t *pDev);

#endif
