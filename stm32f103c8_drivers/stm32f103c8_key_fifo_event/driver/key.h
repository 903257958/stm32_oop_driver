#ifndef KEY_DRV_H
#define KEY_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef TIM_TypeDef* 	timer_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef TIM_TypeDef* 	timer_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;

#else
	#error key.h: No processor defined!
#endif

#endif

#include "timer.h"

#ifndef GPIO_LEVEL_HIGH
	#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
	#define GPIO_LEVEL_LOW 0
#endif

/* 最大按键设备数量 */
#define MAX_KEY_DEV_NUM		6

/* 按键状态标志位 */
#define KEY_DOWN			0x01
#define KEY_UP				0x02
#define KEY_CLICK			0x04
#define KEY_DOUBLE_CLICK	0x08
#define KEY_LONG			0x10
#define KEY_REPEAT			0x20

/* 按键事件类型枚举 */
typedef enum {
    KEY_EVENT_DOWN = 0,
    KEY_EVENT_UP,
    KEY_EVENT_CLICK,
    KEY_EVENT_DOUBLE_CLICK,
    KEY_EVENT_LONG,
    KEY_EVENT_REPEAT,
    KEY_EVENT_MAX
} key_event_type_t;

/* 按键事件回调函数指针 */
typedef void (*key_callback_t)(void *param);

/* 按键事件结构体 */
typedef struct {
    key_callback_t func;	// 回调函数
    void *param;			// 函数形参
} key_event_t;

/* 按键事件映射表，对应每个按键设备 */
typedef struct {
    key_event_t events[KEY_EVENT_MAX];
} key_event_table_t;

/* 按键设备配置结构体 */
typedef struct {
    uint8_t id;                 // 编号
	timer_periph_t timx;		// 用于提供tick，若有多个按键设备传入了不同的定时器外设，则以最后一次初始化的定时器外设为准
	gpio_port_t port;		    // 端口
	gpio_pin_t pin;			    // 引脚
	bool press_level;			// 按键按下的时候IO口的电平
    uint16_t time_ms_double;    // 触发双击时间（单位：毫秒），如果为0则关闭双击检测（启用后需注意每次单击间隔过短会判定为双击）
    uint16_t time_ms_long;      // 触发长按时间（单位：毫秒）
    uint16_t time_ms_repeat;    // 触发重复时间（单位：毫秒）
} key_config_t;

/* 按键设备结构体 */
typedef struct key_dev {
	key_config_t config;
	uint8_t val;								// 按键值，在初始化时被赋值
	bool init_flag;								// 初始化标志
	uint8_t (*get_flag)(struct key_dev *dev);	// 获取按键状态标志位
	void (*clear_flag)(struct key_dev *dev);	// 清除缓冲区中所有按键状态标志位，用于进入新模式时清除历史事件
	bool (*is_pressed)(struct key_dev *dev);	// 按键是否按下
	void (*event_handler)(struct key_dev *dev, key_event_table_t *event_table);	// 按键事件处理
	int8_t (*deinit)(struct key_dev *dev);		// 去初始化
} key_dev_t;

/* 函数声明 */
int8_t key_init(key_dev_t *dev);

#endif
