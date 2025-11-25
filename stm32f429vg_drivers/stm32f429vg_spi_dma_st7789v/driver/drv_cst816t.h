#ifndef DRV_CST816T_H
#define DRV_CST816T_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_CST816T_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*   gpio_port_t;
typedef uint32_t        gpio_pin_t;

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_CST816T_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*   gpio_port_t;
typedef uint32_t        gpio_pin_t;

#elif defined(GD32F10X_MD) || defined(GD32F10X_HD)
#define DRV_CST816T_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t    gpio_port_t;
typedef uint32_t    gpio_pin_t;

#else
#error "drv_cst816t.h: No processor defined!"
#endif

/* ----------------------- 用户配置，可根据实际硬件修改 ----------------------- */
#ifndef MAX_CST816T_NUM
#define MAX_CST816T_NUM 1
#endif

#define CST816T_Y_OFFSET    8
/* -------------------------------------------------------------------------- */

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH 1
#endif

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW  0
#endif

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
typedef enum {
    GESTURE_UP = 0, 
    GESTURE_DOWN,
    GESTURE_LEFT,
    GESTURE_RIGHT
} cst816t_gesture_t;

typedef struct {
    uint16_t x;			        // 当前触摸x坐标
    uint16_t y;			        // 当前触摸y坐标
    cst816t_gesture_t gesture;  // 当前触摸动作
	uint8_t finger_num;	        // 当前触摸手指个数
} cst816t_data_t;

/* I2C 操作接口结构体 */
typedef struct {
	int (*read_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
	int (*read_regs)(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data);
} cst816t_i2c_ops_t;

/* 配置结构体 */
typedef struct {
    const cst816t_i2c_ops_t *i2c_ops;
    void (*delay_ms)(uint32_t ms);
    gpio_port_t rst_port;
	gpio_pin_t  rst_pin;
    uint16_t    screen_x_max;
    uint16_t    screen_y_max;
    bool        is_vertical;    // true: 竖屏；false: 横屏
	bool        is_forward;     // true: 正向；false: 反向
} cst816t_cfg_t;

typedef struct cst816t_dev cst816t_dev_t;

/* 操作接口结构体 */
typedef struct {
    int (*get_id)(cst816t_dev_t *dev, uint8_t *id);
	int (*get_firmware_ver)(cst816t_dev_t *dev, uint8_t *fw_ver);
	int (*get_finger_num)(cst816t_dev_t *dev, uint8_t *finger_num);
    int (*get_data)(cst816t_dev_t *dev, cst816t_data_t *data);
	int (*deinit)(cst816t_dev_t *dev);
} cst816t_ops_t;

/* 设备结构体 */
struct cst816t_dev {
	cst816t_cfg_t cfg;
	const cst816t_ops_t *ops;
};

/**
 * @brief   初始化 CST816T 驱动
 * @param[out] dev cst816t_dev_t 结构体指针
 * @param[in]  cfg cst816t_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_cst816t_init(cst816t_dev_t *dev, const cst816t_cfg_t *cfg);

#endif
