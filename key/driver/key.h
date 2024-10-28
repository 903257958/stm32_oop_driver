#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	Key_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	Key_GPIO_Port;

#else
	#error key.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
	#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
	#define GPIO_LEVEL_LOW 0
#endif

#ifndef key_log
	#define key_log(x) 
#endif

typedef struct {
	Key_GPIO_Port port;						//端口
	uint32_t pin;							//引脚
	bool pressLevel;						//按键按下的时候IO口的电平
}KeyInfo_t;

typedef struct KeyDev {
	KeyInfo_t info;
	bool initFlag;								//初始化标志
	bool (*is_press)(struct KeyDev *pDev);		//判断按键是否按下
	int (*deinit)(struct KeyDev *pDev);			//去初始化
}KeyDev_t;

int key_init(KeyDev_t *pDev);

#endif
