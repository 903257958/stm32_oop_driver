#ifndef __CST816T_H
#define __CST816T_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "i2c.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	CST816T_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	CST816T_GPIO_Port;
	
#else
	#error cst816t.h: No processor defined!
#endif

#ifndef FREERTOS
	#define FREERTOS	0
#endif

#if FREERTOS
	#include "timer.h"
	#include "FreeRTOS.h"
	#include "task.h"

	extern TimerDev_t timerDelay;
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

#ifndef cst816t_log
	#define cst816t_log(x)
#endif

/* LCD屏幕方向选择 */
#define LCD_H				280		// 屏幕高度
#define LCD_W				240		// 屏幕宽度
#define VERTICAL_FORWARD	0		// 竖屏正向
#define VERTICAL_REVERSE	1		// 竖屏反向
#define HORIZONTAL_FORWARD	2		// 横屏正向
#define HORIZONTAL_REVERSE	3		// 横屏反向

/* CST816T的I2C从机地址 */
#define CST816T_ADDRESS	0x2A

/* CST816T寄存器 */
#define GESTURE_ID      	0x01    // 手势寄存器
#define FINGER_NUM      	0x02    // 手指数量
#define X_POS_H         	0x03    // x高四位
#define X_POS_L         	0x04    // x低八位
#define Y_POS_H         	0x05    // y高四位
#define Y_POS_L         	0x06    // y低八位
#define CHIP_ID         	0xA7    // 芯片型号
#define FW_VER				0xA9    // 固件版本
#define MOTION_MASK     	0xEC    // 触发动作
#define AUTO_SLEEP_TIME 	0xF9    // 自动休眠
#define IRQ_CRL         	0xFA    // 中断控制
#define AUTO_RESET      	0xFB    // 无手势休眠
#define LONG_PRESS_TIME 	0xFC    // 长按休眠
#define DIS_AUTO_SLEEP  	0xFE    // 使能低功耗模式

/* CST816T状态码 */
#define GESTURE_UP			1
#define GESTURE_DOWN		2
#define GESTURE_LEFT		3
#define GESTURE_RIGHT		4

typedef struct {
    CST816T_GPIO_Port SCLPort;		// SCL端口
	uint32_t SCLPin;				// SCL引脚
	CST816T_GPIO_Port SDAPort;		// SDA端口
	uint32_t SDAPin;				// SDA引脚
    CST816T_GPIO_Port RSTPort;		// RST端口
	uint32_t RSTPin;				// RST引脚
	uint8_t dir;					// 显示方向
}CST816TInfo_t;

typedef struct CST816TDev {
	CST816TInfo_t info;
	bool initFlag;          // 初始化标志
    void *pPrivData;        // 私有数据指针
	uint16_t x;
    uint16_t y;
    uint8_t gesture;
	void (*get_id)(struct CST816TDev *pDev, uint8_t *id);
	void (*get_firmware_ver)(struct CST816TDev *pDev, uint8_t *fwVer);
	uint8_t (*get_finger_num)(struct CST816TDev *pDev);
    void (*get_action)(struct CST816TDev *pDev);
	int (*deinit)(struct CST816TDev *pDev); // 去初始化
}CST816TDev_t;

int cst816t_init(CST816TDev_t *pDev);

#endif