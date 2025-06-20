#ifndef KEY_DRV_H
#define KEY_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;

#else
	#error key.h: No processor defined!
#endif

#endif

#include "delay.h"

#ifndef KEY_DELAY_MS
	#define KEY_DELAY_MS(ms) delay_ms(ms)
#endif

#ifndef GPIO_LEVEL_HIGH
	#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
	#define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	gpio_port_t port;	// 端口
	gpio_pin_t pin;		// 引脚
	bool press_level;   // 按键按下的时候IO口的电平
} key_config_t;

typedef struct key_dev {
	key_config_t config;
	bool init_flag;								// 初始化标志
	bool (*is_press)(struct key_dev *dev);		// 判断按键是否按下
	int8_t (*deinit)(struct key_dev *dev);		// 去初始化
} key_dev_t;

int8_t key_init(key_dev_t *dev);

#endif
