#ifndef SR04_DRV_H
#define SR04_DRV_H

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

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef*	timer_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;

#else
	#error sr04.h: No processor defined!
#endif

#endif

#include "delay.h"

#ifndef SR04_DELAY_US
	#define SR04_DELAY_US(us)	delay_us(us)
#endif

#ifndef SR04_DELAY_MS
	#define SR04_DELAY_MS(ms)	delay_ms(ms)
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

/* 最大SR04设备数 */
#define MAX_SR04_NUM	1

/* 为避免中断处理函数重复定义，初始化时需根据配置启用对应的中断处理函数 */
#define SR04_TIM2_IRQ_HANDLER_ENABLE	0
#define SR04_TIM3_IRQ_HANDLER_ENABLE	0
#define SR04_TIM4_IRQ_HANDLER_ENABLE	1
#define SR04_TIM5_IRQ_HANDLER_ENABLE	0

/* SR04配置结构体 */
typedef struct {
	timer_periph_t timx;	// 定时器外设，每个SR04设备需要使用不同的定时器
	uint8_t ic_channel;		// 输入捕获通道
	gpio_port_t trig_port;	// Trig端口
	gpio_pin_t trig_pin;	// Trig引脚
	gpio_port_t echo_port;	// Echo端口
	gpio_pin_t echo_pin;	// Echo引脚
} sr04_config_t;

/* SR04设备结构体 */
typedef struct sr04_dev {
	sr04_config_t config;
	bool init_flag;									// 初始化标志
	void *priv_data;								// 私有数据指针
	float distance_cm;								// 距离值
	int8_t (*get_distance)(struct sr04_dev *dev);	// 获取距离值
	int8_t (*deinit)(struct sr04_dev *dev);			// 去初始化
} sr04_dev_t;

/* 函数声明 */
int8_t sr04_init(sr04_dev_t *dev);

#endif
