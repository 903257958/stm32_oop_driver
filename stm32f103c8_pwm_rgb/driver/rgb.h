#ifndef __RGB_H
#define __RGB_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "pwm.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef*	TimerPER_t;
	typedef GPIO_TypeDef*	RGBGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	TimerPER_t;
	typedef GPIO_TypeDef*	RGBGPIOPort_t;

#else
	#error rgb.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	TimerPER_t red_timx;			// R定时器外设
	uint8_t red_oc_channel;			// R输出比较通道
	RGBGPIOPort_t red_port;			// R端口
	uint32_t red_pin;				// R引脚
	TimerPER_t green_timx;			// G定时器外设
	uint8_t green_oc_channel;		// G输出比较通道
	RGBGPIOPort_t green_port;		// G端口
	uint32_t green_pin;				// G引脚
	TimerPER_t blue_timx;			// B定时器外设
	uint8_t blue_oc_channel;		// B输出比较通道
	RGBGPIOPort_t blue_port;		// B端口
	uint32_t blue_pin;				// B引脚
	bool off_level;					// LED灭时IO口的电平
}RGBConfig_t;

typedef struct RGBDev {
	RGBConfig_t config;
	bool init_flag;							// 初始化标志
	void *priv_data;						// 私有数据指针
	int8_t (*set_color)(struct RGBDev *dev, uint8_t red, uint8_t green, uint8_t blue);	// 设置颜色
	int8_t (*red)(struct RGBDev *dev);		// 红色
	int8_t (*yellow)(struct RGBDev *dev);	// 黄色
	int8_t (*green)(struct RGBDev *dev);	// 绿色
	int8_t (*blue)(struct RGBDev *dev);		// 蓝色
	int8_t (*white)(struct RGBDev *dev);	// 白色
	int8_t (*off)(struct RGBDev *dev);		// 关闭
	int8_t (*next_rainbow_color)(struct RGBDev *dev, uint8_t *red, uint8_t *green, uint8_t *blue);	// 下一个彩虹颜色
	int8_t (*deinit)(struct RGBDev *dev);	// 去初始化
}RGBDev_t;

int8_t rgb_init(RGBDev_t *dev);

#endif
