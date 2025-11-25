#ifndef DRV_EEPROM_H
#define DRV_EEPROM_H

#include <stdint.h>
#include <stdbool.h>

/* ----------------------- 用户配置，可根据实际硬件修改 ----------------------- */

/* EEPROM 的 I2C 从机地址 */
#ifndef EEPROM_ADDRESS
#define EEPROM_ADDRESS	0x50	/* AT24C02 */
#endif
/* -------------------------------------------------------------------------- */

/* EEPROM 页大小（字节） */
#define EEPROM_AT24C02_PAGE_SIZE	8	/* AT24C02 */
#define EEPROM_M24C02_PAGE_SIZE		16	/* M24C02 */
// ...

/* I2C 操作接口结构体 */
typedef struct {
	int (*write_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
	int (*write_regs)(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data);
	int (*read_regs)(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data);
} eeprom_i2c_ops_t;

/* 配置结构体 */
typedef struct {
	const eeprom_i2c_ops_t *i2c_ops;
	uint8_t	page_size;	// EEPROM 页大小
} eeprom_cfg_t;

typedef struct eeprom_dev eeprom_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*write_byte)(eeprom_dev_t *dev, uint8_t addr, uint8_t data);
	int (*write_page)(eeprom_dev_t *dev, uint8_t addr, uint8_t *data);
	int (*read_data)(eeprom_dev_t *dev, uint8_t addr, uint16_t num, uint8_t *data);
	int (*deinit)(eeprom_dev_t *dev);
} eeprom_ops_t;

/* 设备结构体 */
struct eeprom_dev {
	eeprom_cfg_t cfg;
	const eeprom_ops_t *ops;
};

/**
 * @brief   初始化 EEPROM 驱动
 * @param[out] dev eeprom_dev_t 结构体指针
 * @param[in]  cfg eeprom_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_eeprom_init(eeprom_dev_t *dev, const eeprom_cfg_t *cfg);

#endif
