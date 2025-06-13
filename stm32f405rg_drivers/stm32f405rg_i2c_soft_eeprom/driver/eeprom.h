#ifndef EEPROM_DRV_H
#define EEPROM_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	eeprom_gpio_port_t;
    typedef uint32_t		eeprom_gpio_pin_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	eeprom_gpio_port_t;
    typedef uint32_t		eeprom_gpio_pin_t;
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
    #include "gd32f10x.h"
    typedef uint32_t    eeprom_gpio_port_t;
    typedef uint32_t	eeprom_gpio_pin_t;
	
#else
    #error eeprom.h: No processor defined!
#endif

#include "i2c_soft.h"

/* EEPROM的I2C从机地址 */
#define EEPROM_ADDRESS	0x50	// AT24C02

/* EEPROM页大小 */
#define EEPROM_PAGE_SIZE 8		// AT24C02
// #define EEPROM_PAGE_SIZE 16		// M24C02

typedef struct {
	eeprom_gpio_port_t scl_port;	// SCL端口
	eeprom_gpio_pin_t scl_pin;		// SCL引脚
	eeprom_gpio_port_t sda_port;	// SDA端口
	eeprom_gpio_pin_t sda_pin;		// SDA引脚
} eeprom_config_t;

typedef struct eeprom_dev {
	eeprom_config_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
	int8_t (*write_byte)(struct eeprom_dev *dev, uint8_t addr, uint8_t data);
	int8_t (*write_page)(struct eeprom_dev *dev, uint8_t addr, uint8_t *data);
	int8_t (*read_data)(struct eeprom_dev *dev, uint8_t addr, uint16_t num, uint8_t *data);
	int8_t (*deinit)(struct eeprom_dev *dev);
} eeprom_dev_t;

int8_t eeprom_init(eeprom_dev_t *dev);

#endif
