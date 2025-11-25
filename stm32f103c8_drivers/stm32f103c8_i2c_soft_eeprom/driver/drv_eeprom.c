#include "drv_eeprom.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 核心驱动层 --------------------------------- */

static int eeprom_write_byte_impl(eeprom_dev_t *dev, uint8_t addr, uint8_t data);
static int eeprom_write_page_impl(eeprom_dev_t *dev, uint8_t addr, uint8_t *data);
static int eeprom_read_data_impl(eeprom_dev_t *dev, uint8_t addr, uint16_t num, uint8_t *data);
static int eeprom_deinit_impl(eeprom_dev_t *dev);

/* 操作接口表 */
static const eeprom_ops_t eeprom_ops = {
	.write_byte = eeprom_write_byte_impl,
	.write_page = eeprom_write_page_impl,
	.read_data  = eeprom_read_data_impl,
	.deinit     = eeprom_deinit_impl
};

/**
 * @brief   初始化 EEPROM 驱动
 * @param[out] dev eeprom_dev_t 结构体指针
 * @param[in]  cfg eeprom_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_eeprom_init(eeprom_dev_t *dev, const eeprom_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	dev->cfg = *cfg;
	dev->ops = &eeprom_ops;
	return 0;
}

/**
 * @brief   EEPROM 写入单字节数据
 * @param[in] dev  eeprom_dev_t 结构体指针
 * @param[in] addr 写入的地址
 * @param[in] data 写入的数据
 * @return	0 表示成功，其他值表示失败
 */
static int eeprom_write_byte_impl(eeprom_dev_t *dev, uint8_t addr, uint8_t data)
{
	if (!dev)
		return -EINVAL;
	
	return dev->cfg.i2c_ops->write_reg(EEPROM_ADDRESS, addr, data);
}

/**
 * @brief   EEPROM 按页写入数据
 * @param[in] dev  eeprom_dev_t 结构体指针
 * @param[in] addr 写入的起始地址
 * @param[in] data 写入的数据
 * @return	0 表示成功，其他值表示失败
 */
static int eeprom_write_page_impl(eeprom_dev_t *dev, uint8_t addr, uint8_t *data)
{
	if (!dev)
		return -EINVAL;

	return dev->cfg.i2c_ops->write_regs(EEPROM_ADDRESS, addr, dev->cfg.page_size, data);
}

/**
 * @brief   EEPROM 读取数据
 * @param[in]  dev  eeprom_dev_t 结构体指针
 * @param[in]  addr 读取的起始地址
 * @param[in]  num  读取数据的个数
 * @param[out] data 读取的数据
 * @return	0 表示成功，其他值表示失败
 */
static int eeprom_read_data_impl(eeprom_dev_t *dev, uint8_t addr, uint16_t num, uint8_t *data)
{
	if (!dev)
		return -EINVAL;
	
	return dev->cfg.i2c_ops->read_regs(EEPROM_ADDRESS, addr, num, data);
}

/**
 * @brief   去初始化 EEPROM 设备
 * @param[in] dev eeprom_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int eeprom_deinit_impl(eeprom_dev_t *dev)
{    
    if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
