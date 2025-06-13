#ifndef AP3216C_DRV_H
#define AP3216C_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	ap3216c_gpio_port_t;
    typedef uint32_t    	ap3216c_gpio_pin_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	ap3216c_gpio_port_t;
    typedef uint32_t    	ap3216c_gpio_pin_t;
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
    #include "gd32f10x.h"
    typedef uint32_t    ap3216c_gpio_port_t;
    typedef uint32_t    ap3216c_gpio_pin_t;
	
#else
    #error ap3216c.h: No processor defined!
#endif

#include "i2c_soft.h"
#include "delay.h"

#ifndef AP3216C_DELAY_MS
	#define AP3216C_DELAY_MS(ms)	delay_ms(ms)
#endif

/* AP3216C的I2C从机地址 */
#define AP3216C_ADDRESS	0x1E

typedef struct {
	ap3216c_gpio_port_t scl_port;	// SCL端口
	ap3216c_gpio_pin_t scl_pin;		// SCL引脚
	ap3216c_gpio_port_t sda_port;	// SDA端口
	ap3216c_gpio_pin_t sda_pin;		// SDA引脚
} ap3216c_config_t;

typedef struct {
	uint16_t light;
    uint16_t proximity;
    uint16_t infrared;
} ap3216c_data_t;

typedef struct ap3216c_dev {
	ap3216c_config_t config;
	ap3216c_data_t data;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
	int8_t (*get_data)(struct ap3216c_dev *dev);
	int8_t (*deinit)(struct ap3216c_dev *dev);
} ap3216c_dev_t;

int8_t ap3216c_init(ap3216c_dev_t *dev);

#endif
