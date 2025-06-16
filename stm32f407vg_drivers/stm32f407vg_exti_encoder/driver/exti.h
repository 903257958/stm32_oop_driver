#ifndef EXTI_DRV_H
#define EXTI_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*		gpio_port_t;
	typedef uint32_t			gpio_pin_t;
	typedef EXTITrigger_TypeDef	exti_trigger_t;

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*		gpio_port_t;
	typedef uint32_t			gpio_pin_t;
	typedef EXTITrigger_TypeDef	exti_trigger_t;

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
	gpio_port_t port;			        // 外部中断端口
	gpio_pin_t pin;			            // 外部中断引脚
	exti_trigger_t trigger;			    // 外部中断触发方式
	uint8_t preemption_priority;	    // 抢占优先级
	uint8_t sub_priority;			    // 响应优先级
	irq_handler_t falling_irq_handler;  // 下降沿外部中断处理函数
    irq_handler_t rising_irq_handler;   // 上升沿外部中断处理函数
} exti_config_t;

/* EXTI设备结构体 */
typedef struct exti_dev {
	exti_config_t config;
	bool init_flag;								// 初始化标志
	int8_t (*deinit)(struct exti_dev *dev);		// 去初始化
} exti_dev_t;

/* 函数声明 */
int8_t exti_init(exti_dev_t *dev);

#endif
