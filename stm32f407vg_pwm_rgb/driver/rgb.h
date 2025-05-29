#ifndef RGB_H
#define RGB_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	timer_periph_t;
	typedef GPIO_TypeDef*	rgb_gpio_port_t;
	typedef uint32_t		rgb_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	timer_periph_t;
	typedef GPIO_TypeDef*	rgb_gpio_port_t;
	typedef uint32_t		rgb_gpio_pin_t;

#else
	#error rgb.h: No processor defined!
#endif

#include "pwm.h"

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	timer_periph_t red_timx;		// R定时器外设
	uint8_t red_oc_channel;			// R输出比较通道
	rgb_gpio_port_t red_port;		// R端口
	rgb_gpio_pin_t red_pin;			// R引脚
	timer_periph_t green_timx;		// G定时器外设
	uint8_t green_oc_channel;		// G输出比较通道
	rgb_gpio_port_t green_port;		// G端口
	rgb_gpio_pin_t green_pin;		// G引脚
	timer_periph_t blue_timx;		// B定时器外设
	uint8_t blue_oc_channel;		// B输出比较通道
	rgb_gpio_port_t blue_port;		// B端口
	rgb_gpio_pin_t blue_pin;		// B引脚
	bool off_level;					// LED灭时IO口的电平
} rgb_config_t;

typedef struct rgb_dev {
	rgb_config_t config;
	bool init_flag;							// 初始化标志
	void *priv_data;						// 私有数据指针
	int8_t (*set_color)(struct rgb_dev *dev, uint8_t red, uint8_t green, uint8_t blue);	// 设置颜色
	int8_t (*red)(struct rgb_dev *dev);		// 红色
	int8_t (*yellow)(struct rgb_dev *dev);	// 黄色
	int8_t (*green)(struct rgb_dev *dev);	// 绿色
	int8_t (*blue)(struct rgb_dev *dev);	// 蓝色
	int8_t (*white)(struct rgb_dev *dev);	// 白色
	int8_t (*off)(struct rgb_dev *dev);		// 关闭
	int8_t (*next_rainbow_color)(struct rgb_dev *dev, uint8_t *red, uint8_t *green, uint8_t *blue);	// 下一个彩虹颜色
	int8_t (*deinit)(struct rgb_dev *dev);	// 去初始化
} rgb_dev_t;

int8_t rgb_init(rgb_dev_t *dev);

#endif
