#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;

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
} gpio_mode_t;

typedef struct {
	gpio_port_t port;	// 端口
	gpio_pin_t pin;		// 引脚
	gpio_mode_t mode;	// 模式
} gpio_config_t;

typedef struct gpio_dev {
	gpio_config_t config;
	bool init_flag;							    		    // 初始化标志
	int8_t (*set)(struct gpio_dev *dev);					// 置位
	int8_t (*reset)(struct gpio_dev *dev);		    		// 复位
	int8_t (*read)(struct gpio_dev *dev, uint8_t *status);	// 读
	int8_t (*write)(struct gpio_dev *dev, uint8_t status);	// 写
	int8_t (*toggle)(struct gpio_dev *dev);		    		// 翻转
	int8_t (*deinit)(struct gpio_dev *dev);		   			// 去初始化
} gpio_dev_t;

int8_t gpio_init(gpio_dev_t *dev);

#endif
