#ifndef DRV_AHT21_H
#define DRV_AHT21_H

#include <stdint.h>
#include <stdbool.h>

#ifndef ETIMEOUT
#define ETIMEOUT	7
#endif

/* AHT21 的 I2C 从机地址 */
#ifndef AHT21_ADDRESS
#define AHT21_ADDRESS	0x38
#endif

/* AHT21 寄存器 */
#define AHT21_GET_STATUS	0x71
#define AHT21_GET_DATA		0x72
#define AHT21_INIT			0xBE
#define AHT21_MEASURE		0xAC

typedef struct {
	float temperature;
	float humidity;
} aht21_data_t;

/* I2C 操作接口结构体 */
typedef struct {
	int (*write_regs)(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data);
	int (*read_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
	int (*read_regs)(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data);
} aht21_i2c_ops_t;

/* 配置结构体 */
typedef struct {
	void (*delay_ms)(uint32_t ms);
	const aht21_i2c_ops_t *i2c_ops;
} aht21_cfg_t;

typedef struct aht21_dev aht21_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*get_data)(aht21_dev_t *dev, aht21_data_t *data);
	int (*deinit)(aht21_dev_t *dev);
} aht21_ops_t;

/* 设备结构体 */
struct aht21_dev {
	aht21_cfg_t cfg;
	const aht21_ops_t *ops;
};

/**
 * @brief   初始化 AHT21 驱动
 * @param[out] dev aht21_dev_t 结构体指针
 * @param[in]  cfg aht21_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_aht21_init(aht21_dev_t *dev, const aht21_cfg_t *cfg);

#endif
