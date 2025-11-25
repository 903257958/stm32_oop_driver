#ifndef DRV_KEY_H
#define DRV_KEY_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_KEY_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_KEY_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_KEY_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_key.h: No processor defined!
#endif

/* ----------------------- 用户配置，可根据实际硬件修改 ----------------------- */
#ifndef MAX_KEY_NUM
#define MAX_KEY_NUM 3   /* 最大按键设备数量 */
#endif

#ifndef KEY_BUF_LEN
#define KEY_BUF_LEN 16  /* 按键环形缓冲区大小 */
#endif
/* -------------------------------------------------------------------------- */

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH	1
#endif

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW 	0
#endif

/* 按键事件类型 */
typedef enum {
    KEY_EVENT_DOWN = 0,
    KEY_EVENT_UP,
    KEY_EVENT_CLICK,
    KEY_EVENT_DOUBLE,
    KEY_EVENT_LONG,
    KEY_EVENT_REPEAT,
    KEY_EVENT_MAX
} key_event_type_t;

/* 按键事件回调函数指针 */
typedef void (*key_event_callback_t)(void *param);

/* 配置结构体 */
typedef struct {
	gpio_port_t port;
	gpio_pin_t  pin;
	bool        press_level;        // 按下后的电平
    uint16_t    debounce_ms;        // 消抖时间（毫秒）
	uint16_t    double_timeout_ms;	// 触发双击时间（毫秒），置 0 则关闭双击检测（启用后需注意每次单击间隔过短会判定为双击）
    uint16_t    long_timeout_ms;    // 触发长按时间（毫秒）
    uint16_t    repeat_timeout_ms;  // 触发重复按时间（毫秒）
    bool        enable_double;      // 使能双击功能
    bool        enable_long;        // 使能长按功能
    bool        enable_repeat;      // 使能重复按功能
} key_cfg_t;

typedef struct key_dev key_dev_t;

/* 操作接口结构体 */
typedef struct {
    int (*register_callback)(key_dev_t *dev, 
                             key_event_type_t type, 
                             key_event_callback_t callback, 
                             void *param);
	int (*process_event)(key_dev_t *dev);
	int (*clear_event)(key_dev_t *dev);
	int (*deinit)(key_dev_t *dev);
} key_ops_t;

/* 设备结构体 */
typedef struct key_dev {
    void *priv;
	key_cfg_t cfg;
	const key_ops_t *ops;
} key_dev_t;

/**
 * @brief   初始化按键设备驱动
 * @param[out] dev key_dev_t 结构体指针
 * @param[in]  cfg key_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_key_init(key_dev_t *dev, const key_cfg_t *cfg);

/**
 * @brief   按键扫描函数，必须 1ms 调用一次
 * @details 每次调用会遍历所有注册的按键，执行以下操作：
 * 			1. 消抖处理；
 * 			2. 记录当前按键状态；
 * 			3. 进行有限状态机（FSM）状态迁移；
 * 			4. 生成对应的按键事件（DOWN / UP / CLICK / DOUBLE / LONG / REPEAT）。
 * @param[in] param 未使用参数（传入 NULL）
 */
void drv_key_tick(void *param);

#endif
