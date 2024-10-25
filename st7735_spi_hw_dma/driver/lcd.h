#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "spi.h"

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

/* LCD屏幕方向选择：0为竖屏正向，1为竖屏反向，2为横屏正向，3为横屏反向 */
#define LCD_DIRECTION	2

#if (LCD_DIRECTION == 0) || (LCD_DIRECTION == 1)
#define LCD_W 128
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 128

#endif

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

/* isFilled参数数值 */
typedef enum {
	LCD_NONUSE_DMA = 0,
	LCD_USE_DMA = 1
}LCDUseDMA_t;

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
	uint16_t prescaler;				// 预分频系数
	SPIMode_t mode;					// SPI模式
	LCDUseDMA_t useDMA;				// 是否使用DMA
}LCDInfo_t;

typedef struct LCDDev {
	LCDInfo_t info;
	bool initFlag;													// 初始化标志
	void *pPrivData;												// 私有数据指针
	void (*update)(struct LCDDev *pDev);							// DMA传输更新显存函数
	void (*fill)(struct LCDDev *pDev, uint16_t color);				// LCD填充颜色
	void (*fill_area)(struct LCDDev *pDev, uint16_t x1, uint16_t y1, uint16_t width, uint16_t height, uint16_t color);						//LCD指定区域填充颜色
	void (*show_char)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t size);					//LCD显示字符
	void (*show_string)(struct LCDDev *pDev, uint16_t x, uint16_t y, char *string, uint16_t fc, uint16_t bc, uint8_t size);					//LCD显示字符串
	void (*show_num)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc , uint8_t size);		//LCD显示整型数字
	void (*show_float_num)(struct LCDDev *pDev, uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size);	//LCD显示两位浮点数
	void (*show_chinese)(struct LCDDev *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t size);				//LCD显示汉字串
	void (*show_image)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);					//LCD显示图片
	void (*draw_point)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t color);														//LCD画点
	void (*draw_line)(struct LCDDev *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);								//LCD画线
	void (*draw_rectangle)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t isFilled);	//LCD画矩形
	void (*draw_circle)(struct LCDDev *pDev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color);										//LCD画圆
	int (*deinit)(struct LCDDev *pDev);																										//去初始化
}LCDDev_t;

int lcd_init(LCDDev_t *pDev);

#endif
