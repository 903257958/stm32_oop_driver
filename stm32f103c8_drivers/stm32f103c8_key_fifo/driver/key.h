#ifndef KEY_DRV_H
#define KEY_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef* 	timer_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef* 	timer_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;

#else
	#error key.h: No processor defined!
#endif

#endif

#include "timer.h"

#ifndef GPIO_LEVEL_HIGH
	#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
	#define GPIO_LEVEL_LOW 0
#endif

/* 最大按键设备数量 */
#define MAX_KEY_DEV_NUM		3

typedef struct {
	timer_periph_t timx;	// 用于提供tick，若有多个按键设备传入了不同的定时器外设，则以最后一次初始化的定时器外设为准
	gpio_port_t port;	    // 端口
	gpio_pin_t pin;		    // 引脚
	bool press_level;		// 按键按下的时候IO口的电平
	int val;				// 按键值
} key_config_t;

typedef struct key_dev {
	key_config_t config;
	bool init_flag;							// 初始化标志
	int8_t (*deinit)(struct key_dev *dev);	// 去初始化
} key_dev_t;

int8_t key_init(key_dev_t *dev);
int key_get_val(void);

#endif
