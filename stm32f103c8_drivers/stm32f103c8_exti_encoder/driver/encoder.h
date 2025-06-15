#ifndef ENCODER_DRV_H
#define ENCODER_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*		encoder_gpio_port_t;
	typedef uint32_t			encoder_gpio_pin_t;

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*		encoder_gpio_port_t;
	typedef uint32_t			encoder_gpio_pin_t;

#else
	#error encoder.h: No processor defined!
#endif

#endif

#include "exti.h"

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

/* 旋转编码器最大数量 */
#define MAX_ENCODER_NUM	2

/* 旋转编码器处理函数指针 */
typedef void (*encoder_callback_t)(void *param);

/* 旋转编码器配置结构体 */
typedef struct {
	encoder_gpio_port_t s1_port;			// S1端口
	encoder_gpio_pin_t s1_pin;				// S1引脚
	encoder_gpio_port_t s2_port;			// S2端口
	encoder_gpio_pin_t s2_pin;				// S2引脚
	encoder_callback_t forward_callback;	// 正转回调函数
	void *forward_callback_param;			// 正转回调函数参数
    encoder_callback_t reverse_callback;	// 反转回调函数
	void *reverse_callback_param;			// 正转回调函数参数
} encoder_config_t;

/* 旋转编码器设备结构体 */
typedef struct encoder_dev {
	encoder_config_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;						    // 私有数据指针
	int8_t (*deinit)(struct encoder_dev *dev);	// 去初始化
} encoder_dev_t;

/* 函数声明 */
int8_t encoder_init(encoder_dev_t *dev);

#endif
