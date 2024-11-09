#ifndef __XPT2046_H
#define __XPT2046_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"

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

typedef struct {
	float xfac;       // 触摸屏与XPT2046的坐标比例系数,  xfac=(float)(20-320)/(t1x-t2x);
    float yfac;                     
    float xoff;       // 像素点偏移值, xoff=(320-xfac*(t1x+t2x))/2;
    float yoff; 
	uint16_t lcdWidth;
    uint16_t lcdHeight;
    uint8_t lcdDir;
}XPT2046Info_t;

typedef struct XPT2046Dev {
    XPT2046Info_t info;
	bool initFlag;							    						// 初始化标志
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
int xpt2046_get_val(void);

#endif
