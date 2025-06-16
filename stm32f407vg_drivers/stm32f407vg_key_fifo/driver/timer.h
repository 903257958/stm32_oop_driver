#ifndef TIMER_DRV_H
#define TIMER_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	timer_periph_t;

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	timer_periph_t;

#else
	#error timer.h: No processor defined!
#endif

#endif

/* 为避免中断处理函数重复定义，初始化时需根据配置启用对应的中断处理函数 */
#define TIM2_IRQ_HANDLER_ENABLE	1
#define TIM3_IRQ_HANDLER_ENABLE	1
#define TIM4_IRQ_HANDLER_ENABLE	1
#define TIM5_IRQ_HANDLER_ENABLE	1
#define TIM6_IRQ_HANDLER_ENABLE	1
#define TIM7_IRQ_HANDLER_ENABLE	1

/* 定时器中断处理函数指针 */
typedef void (*timer_irq_callback_t)(void *param);

/* 定时器配置结构体 */
typedef struct {
	timer_periph_t timx;				// 定时器外设
	uint16_t psc;						// PSC预分频器
	uint16_t arr;						// ARR自动重装器
	timer_irq_callback_t irq_callback;	// 定时器中断处理函数指针
	void *irq_callback_param;			// 定时器中断处理函数参数
	uint8_t preemption_priority;		// 定时器中断抢占优先级
	uint8_t sub_priority;				// 定时器中断响应优先级
} timer_config_t;

/* 定时器设备结构体 */
typedef struct timer_dev {
	timer_config_t config;
	bool init_flag;											// 初始化标志
	int (*delay_us)(struct timer_dev *dev, uint32_t us);	// 微秒级延时，需要正确配置PSC寄存器
	int (*delay_ms)(struct timer_dev *dev, uint32_t ms);	// 毫秒级延时，需要正确配置PSC寄存器
	int8_t (*deinit)(struct timer_dev *dev);				// 去初始化
} timer_dev_t;

/* 函数声明 */
int8_t timer_init(timer_dev_t *dev);

#endif
