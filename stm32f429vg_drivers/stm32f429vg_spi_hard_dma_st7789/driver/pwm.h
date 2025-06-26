#ifndef PWM_DRV_H
#define PWM_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

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
	#error pwm.h: No processor defined!
#endif

#endif

typedef struct {
	timer_periph_t timx;	// 定时器外设
	uint8_t oc_channel;		// 输出比较通道
	uint16_t psc;			// PSC预分频器的值
	uint16_t arr;		    // ARR自动重装器的值
	gpio_port_t port;	    // PWM端口
	gpio_pin_t pin;		    // PWM引脚
} pwm_config_t;

typedef struct pwm_dev {
	pwm_config_t config;
	bool init_flag;												// 初始化标志
	void (*set_psc)(struct pwm_dev *dev, uint16_t psc);			// PWM设置PSC的值
	void (*set_arr)(struct pwm_dev *dev, uint16_t arr);			// PWM设置ARR的值
	void (*set_compare)(struct pwm_dev *dev, uint16_t compare);	// PWM设置CCR的值
	int8_t (*deinit)(struct pwm_dev *dev);						// 去初始化
} pwm_dev_t;

int8_t pwm_init(pwm_dev_t *dev);

#endif
