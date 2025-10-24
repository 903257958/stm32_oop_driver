#ifndef EXTI_DRV_H
#define EXTI_DRV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined (GD32F10X_MD) || defined(GD32F10X_HD)
    #include "gd32f10x.h"
    typedef uint32_t			gpio_port_t;
    typedef uint32_t			gpio_pin_t;
	typedef exti_trig_type_enum	exti_trigger_t;
	
#else
	#error exti.h: No processor defined!
#endif

#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

/* 外部中断处理函数指针 */
typedef void (*irq_handler_t)(void);

/* EXTI配置结构体 */
typedef struct {
	gpio_port_t port;
	gpio_pin_t pin;
	exti_trigger_t trigger;			    // 外部中断触发方式
	uint8_t pre_priority;				// 抢占优先级
	uint8_t sub_priority;				// 响应优先级
	irq_handler_t falling_irq_handler;  // 下降沿外部中断处理函数
    irq_handler_t rising_irq_handler;   // 上升沿外部中断处理函数
} exti_config_t;

/* EXTI设备结构体 */
typedef struct exti_dev {
	exti_config_t config;
	bool init_flag;
	int (*deinit)(struct exti_dev *dev);
} exti_dev_t;

/* 函数声明 */
int exti_drv_init(exti_dev_t *dev);

#endif
