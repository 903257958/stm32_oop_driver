#ifndef VIBRATION_MOTOR_DRV_H
#define VIBRATION_MOTOR_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	vibration_motor_gpio_port_t;
	typedef uint32_t		vibration_motor_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	vibration_motor_gpio_port_t;
	typedef uint32_t		vibration_motor_gpio_pin_t;

#else
    #error vibration_motor.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	vibration_motor_gpio_port_t port;	// 端口
	vibration_motor_gpio_pin_t pin;		// 引脚
} vibration_motor_config_t;

typedef struct vibration_motor_dev {
	vibration_motor_config_t config;
	bool init_flag;							    			// 初始化标志
	void *priv_data;						    			// 私有数据指针
	int8_t (*on)(struct vibration_motor_dev *dev);			// 打开
	int8_t (*off)(struct vibration_motor_dev *dev);			// 关闭
	bool (*get_status)(struct vibration_motor_dev *dev);	// 获取状态
	int8_t (*toggle)(struct vibration_motor_dev *dev);		// 翻转
	int8_t (*deinit)(struct vibration_motor_dev *dev);		// 去初始化
} vibration_motor_dev_t;

int8_t vibration_motor_init(vibration_motor_dev_t *dev);

#endif
