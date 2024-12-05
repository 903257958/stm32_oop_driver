#ifndef __XPT2046_H
#define __XPT2046_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
#include "w25qx.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	XPT2046_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	XPT2046_GPIO_Port;

#else
	#error xpt2046.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
	#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
	#define GPIO_LEVEL_LOW 0
#endif

#ifndef xpt2046_log
	#define xpt2046_log(x) 
#endif

/* Flash设备需在外部定义，且Flash初始化需在XPT2046初始化之前！ */
extern W25QXDev_t flash;

/* 触摸屏方向选择：0为竖屏，1为横屏，需与LCD屏幕方向一致！ */
#define TOUCH_DIRECTION		0

typedef struct XPT2046Dev {
	bool initFlag;							    						// 初始化标志
	uint16_t width;														// 宽度
	uint16_t height;													// 高度
    void *pPrivData;						    						// 私有数据指针
	void (*cmd)(struct XPT2046Dev *pDev, uint8_t status);				// 开关
    bool (*is_pressed)(struct XPT2046Dev *pDev);						// 是否按下
	uint16_t (*get_x)(struct XPT2046Dev *pDev);							// 获取按下位置的x坐标值
	uint16_t (*get_y)(struct XPT2046Dev *pDev);							// 获取按下位置的y坐标值
	int (*recalibration)(struct XPT2046Dev *pDev);						// 重新校准
	void (*draw)(struct XPT2046Dev *pDev, uint16_t color, uint16_t bc);	// 在按下的位置画点
	int (*deinit)(struct XPT2046Dev *pDev);								// 去初始化
}XPT2046Dev_t;

int xpt2046_init(XPT2046Dev_t *pDev);

#endif
