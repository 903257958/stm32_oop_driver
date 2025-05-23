#ifndef __OLED_H
#define __OLED_H

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
	
    typedef GPIO_TypeDef*	OLEDGPIOPort_t;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	OLEDGPIOPort_t;
	
#else
    #error oled.h: No processor defined!
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
	OLEDGPIOPort_t scl_port;		// SCL端口
	uint32_t scl_pin;				// SCL引脚
	OLEDGPIOPort_t sda_port;		// SDA端口
	uint32_t sda_pin;				// SDA引脚
}OLEDConfig_t;

typedef struct OLEDDev {
	OLEDConfig_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
	int8_t (*update)(struct OLEDDev *dev);
	int8_t (*update_area)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int8_t (*clear)(struct OLEDDev *dev);
	int8_t (*clear_area)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int8_t (*reverse)(struct OLEDDev *dev);
	int8_t (*reverse_area)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int8_t (*show_image)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image);
	int8_t (*show_char)(struct OLEDDev *dev, uint8_t x, uint8_t y, char Char, uint8_t FontSize);
	int8_t (*show_string)(struct OLEDDev *dev, uint8_t x, uint8_t y, char *string, uint8_t fontSize);
	int8_t (*show_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int8_t (*show_signed_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, int32_t number, uint8_t length, uint8_t fontSize);
	int8_t (*show_hex_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int8_t (*show_bin_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int8_t (*show_float_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, double number, uint8_t intLength, uint8_t fraLength, uint8_t fontSize);
	int8_t (*show_chinese)(struct OLEDDev *dev, uint8_t x, uint8_t y, char *Chinese);
	int8_t (*printf)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t fontSize, char *format, ...);
	int8_t (*draw_point)(struct OLEDDev *dev, uint8_t x, uint8_t y);
	uint8_t (*get_point)(struct OLEDDev *dev, uint8_t x, uint8_t y);
	int8_t (*draw_line)(struct OLEDDev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	int8_t (*draw_rectangle)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t isFilled);
	int8_t (*draw_triangle)(struct OLEDDev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t IsFilled);
	int8_t (*draw_circle)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t IsFilled);
	int8_t (*draw_ellipse)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t isFilled);
	int8_t (*draw_arc)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t radius, int16_t startAngle, int16_t endAngle, uint8_t isFilled);
	int8_t (*deinit)(struct OLEDDev *dev);
}OLEDDev_t;

int8_t oled_init(OLEDDev_t *dev);

#endif
