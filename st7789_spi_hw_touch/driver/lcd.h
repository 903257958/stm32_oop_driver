#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "spi.h"
#include "pwm.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
	
    typedef GPIO_TypeDef*			LCD_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"

	typedef GPIO_TypeDef*		LCD_GPIO_Port;
	
#else
    #error lcd.h: No processor defined!
#endif

#ifndef FREERTOS
	#define FREERTOS	0
#endif

#if FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

#ifndef lcd_log
    #define lcd_log(x) 
#endif

/* LCD屏幕方向选择 */
#define LCD_H				280		// 屏幕高度
#define LCD_W				240		// 屏幕宽度
#define VERTICAL_FORWARD	0		// 竖屏正向
#define VERTICAL_REVERSE	1		// 竖屏反向
#define HORIZONTAL_FORWARD	2		// 横屏正向
#define HORIZONTAL_REVERSE	3		// 横屏反向

/* LCD画笔颜色 */
typedef enum {
	WHITE		=	0xFFFF,
	BLACK		=	0x0000,
	BLUE		=	0x001F,
	BRED		=	0XF81F,
	GRED		=	0XFFE0,
	GBLUE		=	0X07FF,
	RED			=	0xF800,
	MAGENTA		=	0xF81F,
	GREEN		=	0x07E0,
	CYAN		=	0x7FFF,
	YELLOW		=	0xFFE0,
	BROWN		=	0XBC40,		// 棕色
	BRRED		=	0XFC07,		// 棕红色
	GRAY		=	0X8430,		// 灰色
	DARKBLUE	=	0X01CF,		// 深蓝色
	LIGHTBLUE	=	0X7D7C,		// 浅蓝色
	GRAYBLUE	=	0X5458,		// 灰蓝色
	LIGHTGREEN	=	0X841F,		// 浅绿色
	LGRAY		=	0XC618,		// 浅灰色(PANNEL),窗体背景色
	LGRAYBLUE	=	0XA651,		// 浅灰蓝色(中间层颜色)
	LBBLUE		=	0X2B12		// 浅棕蓝色(选择条目的反色)
}LCDColor_t;

/* LCD显示字体大小 */
typedef enum {
	LCD_16X32 = 32,
	LCD_12X24 = 24,
	LCD_8X16 = 16,
	LCD_6X12 = 12,
	LCD_32X32 = 32,
	LCD_24X24 = 24,
	LCD_16X16 = 16,
	LCD_12X12 = 12
}LCDFontSize_t;

/* isFilled参数数值 */
typedef enum {
	LCD_UNFILLED = 0,
	LCD_FILLED = 1,
}LCDIsFilled_t;

typedef struct {
	SPIx spix;						// SPI外设
	LCD_GPIO_Port RESPort;			// RES端口
	uint32_t RESPin;				// RES引脚
	LCD_GPIO_Port DCPort;			// DC端口
	uint32_t DCPin;					// DC引脚
	LCD_GPIO_Port CSPort;			// CS端口
	uint32_t CSPin;					// CS引脚
	LCD_GPIO_Port BLPort;			// BL端口
	uint32_t BLPin;					// BL引脚
	TIMx timx;						// 背光PWM定时器
	uint8_t OCChannel;				// 背光PWM输出比较通道
	uint8_t dir;					// 显示方向
}LCDInfo_t;

typedef struct LCDDev {
	LCDInfo_t info;
	bool initFlag;													// 初始化标志
	void *pPrivData;												// 私有数据指针
	uint16_t width;
	uint16_t height;
	void (*clear)(struct LCDDev *pDev, uint16_t color);
	void (*fill)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
	void (*color_fill)(struct LCDDev *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *pColor);
	void (*show_char)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_string)(struct LCDDev *pDev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_num)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_hex_num)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_float_num)(struct LCDDev *pDev, uint16_t x, uint16_t y, float num, uint8_t intLen, uint8_t fraLen, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_chinese)(struct LCDDev *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_image)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
	void (*draw_point)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t color);
	void (*draw_line)(struct LCDDev *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
	void (*draw_rectangle)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
	void (*draw_circle)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color);
	int (*deinit)(struct LCDDev *pDev);
}LCDDev_t;

int lcd_init(LCDDev_t *pDev);

#endif
