#ifndef LED_DRV_H
#define LED_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
    #include "gd32f10x.h"
    typedef uint32_t	gpio_port_t;
	typedef uint32_t	gpio_pin_t;

#else
    #error led.h: No processor defined!
#endif

#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

typedef struct {
	gpio_port_t port;
	gpio_pin_t pin;
	bool off_level;	// LED 灭时的电平
} led_config_t;

typedef struct led_dev {
	led_config_t config;
	bool init_flag;
	void *priv_data;
	int (*on)(struct led_dev *dev);
	int (*off)(struct led_dev *dev);
	int (*get_status)(struct led_dev *dev, bool *status);
	int (*toggle)(struct led_dev *dev);
	int (*deinit)(struct led_dev *dev);
} led_dev_t;

int led_drv_init(led_dev_t *dev);

#endif
