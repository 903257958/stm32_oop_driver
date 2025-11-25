#include "drv_aht21.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 核心驱动层 --------------------------------- */

static int aht21_get_data_impl(aht21_dev_t *dev, aht21_data_t *data);
static int aht21_deinit_impl(aht21_dev_t *dev);

/* 操作接口表 */
static const aht21_ops_t aht21_ops = {
	.get_data = aht21_get_data_impl,
	.deinit   = aht21_deinit_impl
};

/**
 * @brief   初始化 AHT21 驱动
 * @param[out] dev aht21_dev_t 结构体指针
 * @param[in]  cfg aht21_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_aht21_init(aht21_dev_t *dev, const aht21_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	uint8_t bits;
	uint8_t datas_init[2] = {0x08, 0x00};

	dev->cfg = *cfg;
	dev->ops = &aht21_ops;

	/* AHT21 初始化 */
	cfg->delay_ms(40);
	cfg->i2c_ops->read_reg(AHT21_ADDRESS, AHT21_GET_STATUS, &bits);
	if (!((bits >> 3) & 0x01)) {
		cfg->i2c_ops->write_regs(AHT21_ADDRESS, AHT21_INIT, 2, datas_init);
		cfg->delay_ms(10);
	}
	return 0;
}

/**
 * @brief   AHT21 获取数据
 * @param[in] dev  aht21_dev_t 结构体指针
 * @param[in] data AHT21 数据
 * @return	0 表示成功，其他值表示失败
 */
static int aht21_get_data_impl(aht21_dev_t *dev, aht21_data_t *data)
{
	if (!dev)
		return -EINVAL;
	
	uint8_t bits, timeout = 100;
	uint8_t datas_measure[2] = {0x33, 0x00};
	uint8_t datas[6] = {0};
	uint32_t temp_t, humi_t;

	/* 触发测量 */
	dev->cfg.i2c_ops->write_regs(AHT21_ADDRESS, AHT21_MEASURE, 2, datas_measure);

	/* 等待测量完成 */
	while (timeout > 0) {
		dev->cfg.i2c_ops->read_reg(AHT21_ADDRESS, AHT21_GET_STATUS, &bits);
		if ((bits & 0x80) == 0)
			break;

		timeout -= 10;
		dev->cfg.delay_ms(10);
	}
	if (timeout == 0)
		return -ETIMEOUT;
	
	/* 读取数据 */
	dev->cfg.i2c_ops->read_regs(AHT21_ADDRESS, AHT21_GET_DATA, 6, datas);

	/* 数据转换 */
	humi_t = (datas[1] << 12) + (datas[2] << 4) + (datas[3] >> 4);		// 拼接完成的20位湿度数据
	data->humidity = (humi_t * 1000 >> 20);								// 湿度数据百分比乘10
	data->humidity = data->humidity / 10;								// 湿度数据百分比
	temp_t = ((datas[3] & 0x0F) << 16) + (datas[4] << 8) + datas[5];	// 拼接完成的20位温度数据
	data->temperature = (temp_t * 2000 >> 20) - 500;					// 温度数据乘10
	data->temperature = data->temperature / 10;							// 温度数据

	return 0;
}

/**
 * @brief   去初始化 AHT21 设备
 * @param[in] dev aht21_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int aht21_deinit_impl(aht21_dev_t *dev)
{    
    if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
