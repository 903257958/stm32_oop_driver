#ifndef AHT21_DRV_H
#define AHT21_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	aht21_gpio_port_t;
	typedef uint32_t		aht21_gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	aht21_gpio_port_t;
	typedef uint32_t		aht21_gpio_pin_t;
	
#else
	#error aht21.h: No processor defined!
#endif

#include "i2c_soft.h"
#include "delay.h"

#ifndef AHT21_DELAY_MS
	#define AHT21_DELAY_MS(ms)	delay_ms(ms)
#endif

/* AHT21的I2C从机地址 */
#define AHT21_ADDRESS		0x38

/* AHT21寄存器 */
#define AHT21_GET_STATUS	0x71
#define AHT21_INIT			0xBE
#define AHT21_MEASURE		0xAC

typedef struct {
    aht21_gpio_port_t scl_port;		// SCL端口
	aht21_gpio_pin_t scl_pin;		// SCL引脚
	aht21_gpio_port_t sda_port;		// SDA端口
	aht21_gpio_pin_t sda_pin;		// SDA引脚
} aht21_config_t;

typedef struct {
    float temperature;
	float humidity;
} aht21_data_t;

typedef struct aht21_dev {
	aht21_config_t config;
	aht21_data_t data;
	bool init_flag;		// 初始化标志
    void *priv_data;	// 私有数据指针
	int8_t (*get_data)(struct aht21_dev *dev);
	int8_t (*deinit)(struct aht21_dev *dev); // 去初始化
} aht21_dev_t;

int8_t aht21_init(aht21_dev_t *dev);

#endif
