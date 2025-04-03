#ifndef __MAX30102_H
#define __MAX30102_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "i2c.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	MAX30102GPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	MAX30102GPIOPort_t;
	
#else
	#error max30102.h: No processor defined!
#endif

/* MAX30102的I2C从机地址 */
#define MAX30102_ADDRESS	0x57

/* MAX30102寄存器 */
#define REG_INTR_STATUS_1 		0x00
#define REG_INTR_STATUS_2 		0x01
#define REG_INTR_ENABLE_1 		0x02
#define REG_INTR_ENABLE_2 		0x03
#define REG_FIFO_WR_PTR 		0x04
#define REG_OVF_COUNTER 		0x05
#define REG_FIFO_RD_PTR 		0x06
#define REG_FIFO_DATA 			0x07
#define REG_FIFO_CONFIG 		0x08
#define REG_MODE_CONFIG 		0x09
#define REG_SPO2_CONFIG 		0x0A
#define REG_LED1_PA 			0x0C
#define REG_LED2_PA 			0x0D
#define REG_PILOT_PA 			0x10
#define REG_MULTI_LED_CTRL1 	0x11
#define REG_MULTI_LED_CTRL2 	0x12
#define REG_TEMP_INTR 			0x1F
#define REG_TEMP_FRAC 			0x20
#define REG_TEMP_CONFIG 		0x21
#define REG_PROX_INT_THRESH 	0x30
#define REG_REV_ID 				0xFE
#define REG_PART_ID 			0xFF

typedef struct {
    MAX30102GPIOPort_t scl_port;	// SCL端口
	uint32_t scl_pin;				// SCL引脚
	MAX30102GPIOPort_t sda_port;	// SDA端口
	uint32_t sda_pin;				// SDA引脚
	MAX30102GPIOPort_t int_port;	// INT端口
	uint32_t int_pin;				// INT引脚
}MAX30102Config_t;

typedef struct MAX30102Dev {
	MAX30102Config_t config;
	bool init_flag;		// 初始化标志
    void *priv_data;	// 私有数据指针
	int32_t heart_rate;
	int32_t blood_oxygen;
	int (*get_data)(struct MAX30102Dev *dev);
	int (*deinit)(struct MAX30102Dev *dev); // 去初始化
}MAX30102Dev_t;

int max30102_init(MAX30102Dev_t *dev);

#endif
