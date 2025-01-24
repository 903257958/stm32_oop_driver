#ifndef __LED_H
#define __LED_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	LED_GPIO_Port;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	LED_GPIO_Port;

#else
    #error led.h: No processor defined!
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

#ifndef led_log
    #define led_log(x) 
#endif

typedef struct {
	LED_GPIO_Port port;						// 端口
	uint32_t pin;							// 引脚
	bool offLevel;							// LED灭时IO口的电平
}LEDInfo_t;

typedef struct LEDDev {
	LEDInfo_t info;
	bool initFlag;							// 初始化标志
	void *pPrivData;						// 私有数据指针
	int (*on)(struct LEDDev *pDev);			// 打开
	int (*off)(struct LEDDev *pDev);		// 关闭
	int (*get_status)(struct LEDDev *pDev);	// 获取状态
	int (*toggle)(struct LEDDev *pDev);		// 翻转
	int (*deinit)(struct LEDDev *pDev);		// 去初始化
}LEDDev_t;

int led_init(LEDDev_t *pDev);

#endif
