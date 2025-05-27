#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pwm.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	TimerPER_t;
	typedef GPIO_TypeDef*	ServoGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	TimerPER_t;
	typedef GPIO_TypeDef*	ServoGPIOPort_t;

#else
	#error servo.h: No processor defined!
#endif

typedef struct {
	TimerPER_t timx;				// 定时器外设
	uint8_t oc_channel;				// 输出比较通道
	ServoGPIOPort_t port;			// Servo端口
	uint32_t pin;					// Servo引脚
}ServoConfig_t;

typedef struct ServoDev {
	ServoConfig_t config;
	bool init_flag;													// 初始化标志
	void *priv_data;												// 私有数据指针
	int8_t (*set_angle)(struct ServoDev *dev, float angle);			// Servo设置角度
	int8_t (*deinit)(struct ServoDev *dev);							// 去初始化
}ServoDev_t;

int8_t servo_init(ServoDev_t *dev);

#endif
