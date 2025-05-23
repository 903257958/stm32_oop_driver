#ifndef __ADC_H
#define __ADC_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	ADCGPIOPort_t;
	typedef ADC_TypeDef *	ADCPER_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	ADCGPIOPort_t;
	typedef ADC_TypeDef *	ADCPER_t;

#else
    #error adc.h: No processor defined!
#endif

typedef struct {
	ADCPER_t adcx;			// ADC外设
	uint8_t channel;		// ADC通道
	ADCGPIOPort_t port;		// 端口
	uint32_t pin;			// 引脚
}ADCConfig_t;

typedef struct ADCDev {
	ADCConfig_t config;
	bool init_flag;										    // 初始化标志
	int8_t (*get_val)(struct ADCDev *dev, uint16_t *val);	// 获取ADC转换值
	int8_t (*deinit)(struct ADCDev *dev);					// 去初始化
}ADCDev_t;

int8_t adc_init(ADCDev_t *dev);

#endif
