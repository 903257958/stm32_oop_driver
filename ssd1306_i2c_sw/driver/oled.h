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
	
    typedef GPIO_TypeDef*	OLED_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	OLED_GPIO_Port;
	
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
	OLED_GPIO_Port scl_port;		// SCL端口
	uint32_t scl_pin;				// SCL引脚
	OLED_GPIO_Port sda_port;		// SDA端口
	uint32_t sda_pin;				// SDA引脚
}OLEDConfig_t;

typedef struct OLEDDev {
	OLEDConfig_t config;
	bool init_flag;								// 初始化标志
	void *priv_data;							// 私有数据指针
	int (*update)(struct OLEDDev *dev);
	int (*update_area)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*clear)(struct OLEDDev *dev);
	int (*clear_area)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*reverse)(struct OLEDDev *dev);
	int (*reverse_area)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*show_image)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image);
	int (*show_char)(struct OLEDDev *dev, uint8_t x, uint8_t y, char Char, uint8_t FontSize);
	int (*show_string)(struct OLEDDev *dev, uint8_t x, uint8_t y, char *string, uint8_t fontSize);
	int (*show_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int (*show_signed_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, int32_t number, uint8_t length, uint8_t fontSize);
	int (*show_hex_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int (*show_bin_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
	int (*show_float_num)(struct OLEDDev *dev, uint8_t x, uint8_t y, double number, uint8_t intLength, uint8_t fraLength, uint8_t fontSize);
	int (*show_chinese)(struct OLEDDev *dev, uint8_t x, uint8_t y, char *Chinese);
	int (*printf)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t fontSize, char *format, ...);
	int (*draw_point)(struct OLEDDev *dev, uint8_t x, uint8_t y);
	uint8_t (*get_point)(struct OLEDDev *dev, uint8_t x, uint8_t y);
	int (*draw_line)(struct OLEDDev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	int (*draw_rectangle)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t isFilled);
	int (*draw_triangle)(struct OLEDDev *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t IsFilled);
	int (*draw_circle)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t IsFilled);
	int (*draw_ellipse)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t isFilled);
	int (*draw_arc)(struct OLEDDev *dev, uint8_t x, uint8_t y, uint8_t radius, int16_t startAngle, int16_t endAngle, uint8_t isFilled);
	int (*deinit)(struct OLEDDev *dev);
}OLEDDev_t;

int oled_init(OLEDDev_t *dev);

#endif
