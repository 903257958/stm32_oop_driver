#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
	
#if defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	LCD_GPIO_Port;

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

/* LCD屏幕方向选择：0为竖屏，1为横屏 */
#define LCD_DIRECTION	0

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
	BROWN		=	0XBC40,	// 棕色
	BRRED		=	0XFC07,	// 棕红色
	GRAY		=	0X8430,	// 灰色
	DARKBLUE	=	0X01CF,	// 深蓝色
	LIGHTBLUE	=	0X7D7C,	// 浅蓝色
	GRAYBLUE	=	0X5458,	// 灰蓝色
	LIGHTGREEN	=	0X841F,	// 浅绿色
	LGRAY		=	0XC618,	// 浅灰色(PANNEL),窗体背景色
	LGRAYBLUE	=	0XA651,	// 浅灰蓝色(中间层颜色)
	LBBLUE		=	0X2B12	// 浅棕蓝色(选择条目的反色)
}LCDColor_t;

/* LCD屏幕的引脚一般不会改变，直接在 lcd_init 函数中的私有数据初始化中定义引脚，不提供外部接口 */
typedef struct LCDDev {
	bool initFlag;							// 初始化标志
	uint16_t width;							// 宽度
	uint16_t height;						// 高度
	void *pPrivData;						// 私有数据指针
	void (*clear)(struct LCDDev *pDev, uint16_t color);
	void (*fill)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
	void (*show_char)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_string)(struct LCDDev *pDev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_num)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_float_num)(struct LCDDev *pDev, uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_chinese)(struct LCDDev *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
	void (*show_image)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
	void (*draw_point)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t color);
	uint16_t (*read_point)(struct LCDDev *pDev, uint16_t x, uint16_t y);
	void (*draw_line)(struct LCDDev *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
	void (*draw_rectangle)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
	void (*draw_circle)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t r, uint16_t color);
	int (*deinit)(struct LCDDev *pDev);		// 去初始化
}LCDDev_t;

int lcd_init(LCDDev_t *pDev);

#endif
