#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "timer.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	KeyGPIOPort_t;
	typedef TIM_TypeDef* 	TimerPER_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	KeyGPIOPort_t;
	typedef TIM_TypeDef* 	TimerPER_t;

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
	TimerPER_t timx;			// 用于提供tick，若有多个按键设备传入了不同的定时器外设，则以最后一次初始化的定时器外设为准
	KeyGPIOPort_t port;			// 端口
	uint32_t pin;				// 引脚
	bool press_level;			// 按键按下的时候IO口的电平
	int val;					// 按键值
}KeyConfig_t;

typedef struct KeyDev {
	KeyConfig_t config;
	bool init_flag;							// 初始化标志
	int8_t (*deinit)(struct KeyDev *dev);	// 去初始化
}KeyDev_t;

int8_t key_init(KeyDev_t *dev);
int key_get_val(void);

#endif
