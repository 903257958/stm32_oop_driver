#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	SPIGPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	SPIGPIOPort_t;
	
#else
	#error spi.h: No processor defined!
#endif

typedef enum {
	SPI_MODE_0,
	SPI_MODE_1,
	SPI_MODE_2,
	SPI_MODE_3
}SPIMode_t;

typedef struct {
	SPIGPIOPort_t sck_port;		// SCK端口
	uint32_t sck_pin;			// SCK引脚
	SPIGPIOPort_t miso_port;	// MISO端口
	uint32_t miso_pin;			// MISO引脚
	SPIGPIOPort_t mosi_port;	// MOSI端口
	uint32_t mosi_pin;			// MOSI引脚
	SPIGPIOPort_t cs_port;		// CS端口
	uint32_t cs_pin;			// CS引脚
	SPIMode_t mode;				// SPI模式
}SPIConfig_t;

typedef struct SPIDev_t {
	SPIConfig_t config;
	bool init_flag;													// 初始化标志
	void (*sck_write)(struct SPIDev_t *dev, uint8_t bit_val);		// 软件SPI写SCK引脚电平
	void (*mosi_write)(struct SPIDev_t *dev, uint8_t bit_val);		// 软件SPI写MOSI引脚电平
	uint8_t (*miso_read)(struct SPIDev_t *dev);						// 软件SPI读MISO引脚电平
	void (*cs_write)(struct SPIDev_t *dev, uint8_t bit_val);		// 软件SPI写CS引脚电平
	void (*start)(struct SPIDev_t *dev);							// 软件SPI起始
	void (*stop)(struct SPIDev_t *dev);								// 软件SPI停止
	uint8_t (*swap_byte)(struct SPIDev_t *dev, uint8_t send_byte);	// 软件SPI交换一个字节
	int (*deinit)(struct SPIDev_t *dev);							// 去初始化
}SPIDev_t;

int spi_init(SPIDev_t *dev);
#endif
