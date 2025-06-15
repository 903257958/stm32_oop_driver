#ifndef TB6612_DRV_H
#define TB6612_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	timer_periph_t;
	typedef GPIO_TypeDef*	tb6612_gpio_port_t;
	typedef uint32_t		tb6612_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	timer_periph_t;
	typedef GPIO_TypeDef*	tb6612_gpio_port_t;
	typedef uint32_t		tb6612_gpio_pin_t;

#else
	#error tb6612.h: No processor defined!
#endif

#endif

#include "pwm.h"

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef enum {
	MOTOR_1 = 1,
	MOTOR_2 = 2
} motor_id_t;

typedef struct {
	timer_periph_t pwma_timx;		// PWMA定时器外设
	uint8_t pwma_oc_channel;		// PWMA输出比较通道
	tb6612_gpio_port_t pwma_port;	// PWMA端口
	tb6612_gpio_pin_t pwma_pin;		// PWMA引脚
	tb6612_gpio_port_t ain1_port;	// AIN1端口
	tb6612_gpio_pin_t ain1_pin;		// AIN1引脚
	tb6612_gpio_port_t ain2_port;	// AIN2端口
	tb6612_gpio_pin_t ain2_pin;		// AIN2引脚
	timer_periph_t pwmb_timx;		// PWMB定时器外设
	uint8_t pwmb_oc_channel;		// PWMB输出比较通道
	tb6612_gpio_port_t pwmb_port;	// PWMB端口
	tb6612_gpio_pin_t pwmb_pin;		// PWMB引脚
	tb6612_gpio_port_t bin1_port;	// BIN1端口
	tb6612_gpio_pin_t bin1_pin;		// BIN1引脚
	tb6612_gpio_port_t bin2_port;	// BIN2端口
	tb6612_gpio_pin_t bin2_pin;		// BIN2引脚
} tb6612_config_t;

typedef struct tb6612_dev {
	tb6612_config_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
	int8_t (*set_speed)(struct tb6612_dev *dev, motor_id_t motor_id, int8_t speed);	// 设置速度
	int8_t (*deinit)(struct tb6612_dev *dev);	// 去初始化
} tb6612_dev_t;

int8_t tb6612_init(tb6612_dev_t *dev);

#endif
