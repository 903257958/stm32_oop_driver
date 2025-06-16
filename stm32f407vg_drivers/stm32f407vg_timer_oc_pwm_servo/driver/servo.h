#ifndef SERVO_DRV_H
#define SERVO_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	timer_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	timer_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;

#else
	#error servo.h: No processor defined!
#endif

#endif

#include "pwm.h"

typedef struct {
	timer_periph_t timx;	// 定时器外设
	uint8_t oc_channel;		// 输出比较通道
	gpio_port_t port;	    // Servo端口
	gpio_pin_t pin;	        // Servo引脚
} servo_config_t;

typedef struct servo_dev {
	servo_config_t config;
	bool init_flag;												// 初始化标志
	void *priv_data;											// 私有数据指针
	int8_t (*set_angle)(struct servo_dev *dev, float angle);	// Servo设置角度
	int8_t (*deinit)(struct servo_dev *dev);					// 去初始化
} servo_dev_t;

int8_t servo_init(servo_dev_t *dev);

#endif
