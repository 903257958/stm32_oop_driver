#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	SPIGPIOPort_t;
	typedef SPI_TypeDef *	SPIPER_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	SPIGPIOPort_t;
	typedef SPI_TypeDef *	SPIPER_t;
	
#else
	#error spi.h: No processor defined!
#endif

#ifndef spi_log
	#define spi_log(x) 
#endif

typedef enum {
    SPI_MODE_0,
    SPI_MODE_1,
    SPI_MODE_2,
    SPI_MODE_3
}SPIMode_t;

typedef struct {
	SPIPER_t spix;				// SPI外设
	SPIGPIOPort_t sck_port;		// SCK端口
	uint32_t sck_pin;			// SCK引脚
	SPIGPIOPort_t mosi_port;	// MOSI端口
	uint32_t mosi_pin;			// MOSI引脚
	SPIGPIOPort_t miso_port;	// MISO端口
	uint32_t miso_pin;			// MISO引脚
	SPIGPIOPort_t cs_port;		// CS端口
	uint32_t cs_pin;			// CS引脚
	uint16_t prescaler;			// 预分频系数
	SPIMode_t mode;				// SPI模式
}SPIInfo_t;

typedef struct SPIDev_t {
	SPIInfo_t info;
	bool init_flag;														// 初始化标志
	void (*cs_write)(struct SPIDev_t *dev, uint8_t bit_val);			// 硬件SPI写CS引脚电平
	void (*start)(struct SPIDev_t *dev);								// 硬件SPI起始
	void (*stop)(struct SPIDev_t *dev);									// 硬件SPI停止
	uint8_t (*swap_byte)(struct SPIDev_t *dev, uint8_t send_byte);		// 硬件SPI交换一个字节
	int (*deinit)(struct SPIDev_t *dev);								// 去初始化
}SPIDev_t;

int spi_init(SPIDev_t *dev);

#endif
