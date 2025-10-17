#ifndef TIMER_DRV_H
#define TIMER_DRV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER
	
#if defined (GD32F10X_MD)
    #include "gd32f10x.h"
    typedef uint32_t	timer_periph_t;
#else
	#error timer.h: No processor defined!
#endif

#endif

/* 为避免中断处理函数重复定义，初始化时需根据配置启用对应的中断处理函数 */
#define TIM1_IRQ_HANDLER_ENABLE	1
#define TIM2_IRQ_HANDLER_ENABLE	1
#define TIM3_IRQ_HANDLER_ENABLE	1

/* 定时器中断处理函数指针 */
typedef void(*timer_irq_callback_t)(void *arg);

/* 定时器配置结构体 */
typedef struct {
	timer_periph_t timx;
	uint16_t psc;
	uint16_t arr;
	timer_irq_callback_t irq_callback;
	void *irq_callback_arg;
	uint8_t pre_priority;	// 定时器中断抢占优先级
	uint8_t sub_priority;	// 定时器中断响应优先级
} timer_config_t;

/* 定时器设备结构体 */
typedef struct timer_dev {
	timer_config_t config;
	bool init_flag;
	int (*delay_us)(struct timer_dev *dev, uint32_t us);
	int (*delay_ms)(struct timer_dev *dev, uint32_t ms);
	int (*deinit)(struct timer_dev *dev);
} timer_dev_t;

/* 函数声明 */
int timer_drv_init(timer_dev_t *dev);

#endif
