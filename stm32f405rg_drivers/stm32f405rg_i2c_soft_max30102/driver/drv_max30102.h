#ifndef DRV_MAX30102_H
#define DRV_MAX30102_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_MAX30102_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_MAX30102_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_MAX30102_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_max30102.h: No processor defined!
#endif

/* MAX30102 的 I2C 从机地址 */
#ifndef MAX30102_ADDRESS
#define MAX30102_ADDRESS			0x57
#endif

/* MAX30102 寄存器 */
#define MAX30102_INTR_STATUS_1		0x00
#define MAX30102_INTR_STATUS_2 		0x01
#define MAX30102_INTR_ENABLE_1 		0x02
#define MAX30102_INTR_ENABLE_2 		0x03
#define MAX30102_FIFO_WR_PTR 		0x04
#define MAX30102_OVF_COUNTER 		0x05
#define MAX30102_FIFO_RD_PTR 		0x06
#define MAX30102_FIFO_DATA 			0x07
#define MAX30102_FIFO_CONFIG 		0x08
#define MAX30102_MODE_CONFIG 		0x09
#define MAX30102_SPO2_CONFIG 		0x0A
#define MAX30102_LED1_PA 			0x0C
#define MAX30102_LED2_PA 			0x0D
#define MAX30102_PILOT_PA 			0x10
#define MAX30102_MULTI_LED_CTRL1	0x11
#define MAX30102_MULTI_LED_CTRL2	0x12
#define MAX30102_TEMP_INTR 			0x1F
#define MAX30102_TEMP_FRAC 			0x20
#define MAX30102_TEMP_CONFIG 		0x21
#define MAX30102_PROX_INT_THRESH 	0x30
#define MAX30102_REV_ID				0xFE
#define MAX30102_PART_ID 			0xFF

typedef struct {
    int32_t heart_rate;
	int32_t blood_oxygen;
} max30102_data_t;

/* I2C 操作接口结构体 */
typedef struct {
	int (*write_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
	int (*read_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
	int (*read_regs)(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data);
} max30102_i2c_ops_t;

/* 配置结构体 */
typedef struct {
	const max30102_i2c_ops_t *i2c_ops;
	gpio_port_t irq_port;
	gpio_pin_t irq_pin;
} max30102_cfg_t;

typedef struct max30102_dev max30102_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*soft_init)(max30102_dev_t *dev);
	int (*get_data)(max30102_dev_t *dev, max30102_data_t *data);
	int (*deinit)(max30102_dev_t *dev);
} max30102_ops_t;

/* 设备结构体 */
struct max30102_dev {
	max30102_cfg_t cfg;
	const max30102_ops_t *ops;
};

/**
 * @brief   初始化 MAX30102 驱动
 * @param[out] dev max30102_dev_t 结构体指针
 * @param[in]  cfg max30102_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_max30102_init(max30102_dev_t *dev, const max30102_cfg_t *cfg);

#endif
