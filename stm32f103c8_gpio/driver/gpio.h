#ifndef __GPIO_H
#define __GPIO_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	GPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	GPIOPort_t;

#else
    #error gpio.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef enum {
	GPIO_MODE_IN_PU = 1,	// 上拉输入
	GPIO_MODE_IN_PD,		// 下拉输入
	GPIO_MODE_IN_PN,		// 浮空输入
	GPIO_MODE_OUT_PP,		// 推挽输出
	GPIO_MODE_OUT_OD,		// 开漏输出
}GPIOMode_t;

typedef struct {
	GPIOPort_t port;    // 端口
	uint32_t pin;       // 引脚
	GPIOMode_t mode;	// 模式
}GPIOConfig_t;

typedef struct GPIODev {
	GPIOConfig_t config;
	bool init_flag;							    		    // 初始化标志
	int8_t (*set)(struct GPIODev *dev);					    // 置位
	int8_t (*reset)(struct GPIODev *dev);		    		// 复位
	int8_t (*read)(struct GPIODev *dev, uint8_t *status);	// 读
	int8_t (*write)(struct GPIODev *dev, uint8_t status);	// 写
	int8_t (*toggle)(struct GPIODev *dev);		    		// 翻转
	int8_t (*deinit)(struct GPIODev *dev);		   			// 去初始化
}GPIODev_t;

int8_t gpio_init(GPIODev_t *dev);

#endif
