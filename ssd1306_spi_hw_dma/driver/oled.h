#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "spi.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
	
    typedef GPIO_TypeDef*	OLED_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	OLED_GPIO_Port;
	
#else
    #error oled.h: No processor defined!
#endif

#ifndef OLED_Log
    #define OLED_Log(x) 
#endif

/* 屏幕方向选择，0为正向，1为反向 */
#define OLED_DIRECTION		0

/* fontSize参数取值 */
/* 此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度 */
#define OLED_8X16				8
#define OLED_6X8				6

/* isFilled参数数值 */
#define OLED_UNFILLED			0
#define OLED_FILLED				1

typedef struct {
	SPIx spix;						// SPI外设
	OLED_GPIO_Port RESPort;			// RES端口
	uint32_t RESPin;				// RES引脚
	OLED_GPIO_Port DCPort;			// DC端口
	uint32_t DCPin;					// DC引脚
	OLED_GPIO_Port CSPort;			// CS端口
	uint32_t CSPin;					// CS引脚
	uint16_t prescaler;				// 预分频系数
	SPIMode_t mode;					// SPI模式
}OLEDInfo_t;

typedef struct OLEDDev {
	OLEDInfo_t info;
	bool initFlag;								// 初始化标志
	void *pPrivData;							// 私有数据指针
	int (*update)(struct OLEDDev *pDev);
	int (*update_area)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*clear)(struct OLEDDev *pDev);
	int (*clear_area)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*reverse)(struct OLEDDev *pDev);
	int (*reverse_area)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*show_image)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image);
	int (*show_char)(struct OLEDDev *pDev, uint8_t x, uint8_t y, char Char, uint8_t FontSize);
	int (*show_string)(struct OLEDDev *pDev, uint8_t x, uint8_t y, char *string, uint8_t fontSize);
	int (*show_num)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int (*show_signed_num)(struct OLEDDev *pDev, uint8_t x, uint8_t y, int32_t number, uint8_t length, uint8_t fontSize);
	int (*show_hex_num)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int (*show_bin_num)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int (*show_float_num)(struct OLEDDev *pDev, uint8_t x, uint8_t y, double number, uint8_t intLength, uint8_t fraLength, uint8_t fontSize);
	int (*show_chinese)(struct OLEDDev *pDev, uint8_t x, uint8_t y, char *Chinese);
	int (*printf)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t fontSize, char *format, ...);
	int (*draw_point)(struct OLEDDev *pDev, uint8_t x, uint8_t y);
	uint8_t (*get_point)(struct OLEDDev *pDev, uint8_t x, uint8_t y);
	int (*draw_line)(struct OLEDDev *pDev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	int (*draw_rectangle)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t isFilled);
	int (*draw_triangle)(struct OLEDDev *pDev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t IsFilled);
	int (*draw_circle)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t radius, uint8_t IsFilled);
	int (*draw_ellipse)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t isFilled);
	int (*draw_arc)(struct OLEDDev *pDev, uint8_t x, uint8_t y, uint8_t radius, int16_t startAngle, int16_t endAngle, uint8_t isFilled);
	int (*deinit)(struct OLEDDev *pDev);
}OLEDDev_t;

int oled_init(OLEDDev_t *pDev);

#endif
