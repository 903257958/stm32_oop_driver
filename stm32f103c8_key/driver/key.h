#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	KeyGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	KeyGPIOPort_t;

#else
	#error key.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
	#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
	#define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	KeyGPIOPort_t port;						// 端口
	uint32_t pin;							// 引脚
	bool press_level;						// 按键按下的时候IO口的电平
}KeyConfig_t;

typedef struct KeyDev {
	KeyConfig_t config;
	bool init_flag;								// 初始化标志
	bool (*is_press)(struct KeyDev *dev);		// 判断按键是否按下
	int8_t (*deinit)(struct KeyDev *dev);		// 去初始化
}KeyDev_t;

int8_t key_init(KeyDev_t *dev);

#endif
