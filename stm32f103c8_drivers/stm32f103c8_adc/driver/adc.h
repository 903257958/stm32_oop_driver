#ifndef ADC_DRV_H
#define ADC_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef ADC_TypeDef*	adc_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef ADC_TypeDef*	adc_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;

#else
    #error adc.h: No processor defined!
#endif

#endif

typedef struct {
	adc_periph_t adcx;  // ADC外设
	uint8_t channel;    // ADC通道
	gpio_port_t port;	// 端口
	gpio_pin_t pin;		// 引脚
} adc_config_t;

typedef struct adc_dev {
	adc_config_t config;
	bool init_flag;										    // 初始化标志
	int8_t (*get_val)(struct adc_dev *dev, uint16_t *val);	// 获取ADC转换值
	int8_t (*deinit)(struct adc_dev *dev);					// 去初始化
} adc_dev_t;

int8_t adc_init(adc_dev_t *dev);

#endif
