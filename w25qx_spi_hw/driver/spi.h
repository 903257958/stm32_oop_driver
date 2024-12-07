#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	SPI_GPIO_Port;
	typedef SPI_TypeDef *	SPIx;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	SPI_GPIO_Port;
	typedef SPI_TypeDef *	SPIx;
	
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
	SPIx spix;					// SPI外设
	SPI_GPIO_Port SCKPort;		// SCK端口
	uint32_t SCKPin;			// SCK引脚
	SPI_GPIO_Port MOSIPort;		// MOSI端口
	uint32_t MOSIPin;			// MOSI引脚
	SPI_GPIO_Port MISOPort;		// MISO端口
	uint32_t MISOPin;			// MISO引脚
	SPI_GPIO_Port CSPort;		// CS端口
	uint32_t CSPin;				// CS引脚
	uint16_t prescaler;			// 预分频系数
	SPIMode_t mode;				// SPI模式
}SPIInfo_t;

typedef struct SPIDev_t {
	SPIInfo_t info;
	bool initFlag;														// 初始化标志
	void (*cs_write)(struct SPIDev_t *pDev, uint8_t bitValue);			// 硬件SPI写CS引脚电平
	void (*start)(struct SPIDev_t *pDev);								// 硬件SPI起始
	void (*stop)(struct SPIDev_t *pDev);								// 硬件SPI停止
	uint8_t (*swap_byte)(struct SPIDev_t *pDev, uint8_t sendByte);		// 硬件SPI交换一个字节
	int (*deinit)(struct SPIDev_t *pDev);								// 去初始化
}SPIDev_t;

int spi_init(SPIDev_t *pDev);
#endif
