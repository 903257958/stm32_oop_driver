#ifndef CST816T_DRV_H
#define CST816T_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#else
	#error cst816t.h: No processor defined!
#endif

#endif

#include "i2c_soft.h"
#include "delay.h"

#ifndef CST816T_DELAY_MS
	#define CST816T_DELAY_MS(ms)	delay_ms(ms)
#endif

#ifndef GPIO_LEVEL_HIGH
    #define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
    #define GPIO_LEVEL_LOW 0
#endif

/* LCD屏幕尺寸 */
#define LCD_H				280		// 屏幕高度
#define LCD_W				240		// 屏幕宽度

/* LCD屏幕方向选择 */
#define VERTICAL_FORWARD	0		// 竖屏正向
#define VERTICAL_REVERSE	1		// 竖屏反向
#define HORIZONTAL_FORWARD	2		// 横屏正向
#define HORIZONTAL_REVERSE	3		// 横屏反向

/* CST816T的I2C从机地址 */
#define CST816T_ADDRESS		0x15

/* CST816T寄存器 */
#define GESTURE_ID      	0x01    // 手势寄存器
#define FINGER_NUM      	0x02    // 手指数量
#define X_POS_H         	0x03    // x高四位
#define X_POS_L         	0x04    // x低八位
#define Y_POS_H         	0x05    // y高四位
#define Y_POS_L         	0x06    // y低八位
#define CHIP_ID         	0xA7    // 芯片型号
#define FW_VER				0xA9    // 固件版本
#define SLEEP_MODE     		0xE5    // 睡眠模式
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
    gpio_port_t scl_port;	// SCL端口
	gpio_pin_t scl_pin;		// SCL引脚
	gpio_port_t sda_port;	// SDA端口
	gpio_pin_t sda_pin;		// SDA引脚
    gpio_port_t rst_port;	// RST端口
	gpio_pin_t rst_pin;		// RST引脚
	uint8_t dir;			// 显示方向
} cst816t_config_t;

typedef struct {
    uint16_t x;			// 当前触摸x坐标
    uint16_t y;			// 当前触摸y坐标
    uint8_t gesture;	// 当前触摸动作
	uint8_t finger_num;	// 当前触摸手指个数
} cst816t_data_t;

typedef struct cst816t_dev {
	cst816t_config_t config;
	cst816t_data_t data;
	bool init_flag;		// 初始化标志
    void *priv_data;	// 私有数据指针
	int8_t (*get_id)(struct cst816t_dev *dev, uint8_t *id);
	int8_t (*get_firmware_ver)(struct cst816t_dev *dev, uint8_t *fw_ver);
	int8_t (*get_finger_num)(struct cst816t_dev *dev);
    int8_t (*get_action)(struct cst816t_dev *dev);
	int8_t (*deinit)(struct cst816t_dev *dev); // 去初始化
} cst816t_dev_t;

int8_t cst816t_init(cst816t_dev_t *dev);

#endif
