#ifndef __LED_H
#define __LED_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	LEDGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	LEDGPIOPort_t;

#else
    #error led.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	LEDGPIOPort_t port;						// 端口
	uint32_t pin;							// 引脚
	bool off_level;							// LED灭时IO口的电平
}LEDConfig_t;

typedef struct LEDDev {
	LEDConfig_t config;
	bool init_flag;							// 初始化标志
	void *priv_data;						// 私有数据指针
	int (*on)(struct LEDDev *dev);			// 打开
	int (*off)(struct LEDDev *dev);			// 关闭
	int (*get_status)(struct LEDDev *dev);	// 获取状态
	int (*toggle)(struct LEDDev *dev);		// 翻转
	int (*deinit)(struct LEDDev *dev);		// 去初始化
}LEDDev_t;

int led_init(LEDDev_t *dev);

#endif
