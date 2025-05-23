#ifndef __EEPROM_H
#define __EEPROM_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "i2c.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
    typedef GPIO_TypeDef*	EEPROMGPIOPort_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	EEPROMGPIOPort_t;
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
    #include "gd32f10x.h"
    typedef uint32_t    EEPROMGPIOPort_t;
	
#else
    #error eeprom.h: No processor defined!
#endif

/* EEPROM的I2C从机地址 */
#define EEPROM_ADDRESS	0x50	// AT24C02

/* EEPROM页大小 */
#define EEPROM_PAGE_SIZE 8		// AT24C02
// #define EEPROM_PAGE_SIZE 16		// M24C02

typedef struct {
	EEPROMGPIOPort_t scl_port;		// SCL端口
	uint32_t scl_pin;				// SCL引脚
	EEPROMGPIOPort_t sda_port;		// SDA端口
	uint32_t sda_pin;				// SDA引脚
}EEPROMConfig_t;

typedef struct EEPROMDev {
	EEPROMConfig_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
	int8_t (*write_byte)(struct EEPROMDev *dev, uint8_t addr, uint8_t data);
	int8_t (*write_page)(struct EEPROMDev *dev, uint8_t addr, uint8_t *data);
	int8_t (*read_data)(struct EEPROMDev *dev, uint8_t addr, uint16_t num, uint8_t *data);
	int8_t (*deinit)(struct EEPROMDev *dev);
}EEPROMDev_t;

int8_t eeprom_init(EEPROMDev_t *dev);

#endif
