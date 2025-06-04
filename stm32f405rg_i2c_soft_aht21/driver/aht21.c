#include "aht21.h"

/* AHT21私有数据结构体 */
typedef struct {
	i2c_soft_dev_t i2c;	// 软件I2C设备
} aht21_priv_data_t;

/* 函数声明 */
static int8_t __aht21_get_data(aht21_dev_t *dev);
static int8_t __aht21_deinit(aht21_dev_t *dev);

/******************************************************************************
 * @brief	初始化AHT21
 * @param	dev	:  aht21_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t aht21_init(aht21_dev_t *dev)
{
    if (!dev)
		return -1;

	uint8_t bits;
	uint8_t datas_init[2] = {0x08, 0x00};

    /* 保存私有数据 */
	dev->priv_data = (aht21_priv_data_t *)malloc(sizeof(aht21_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	aht21_priv_data_t *priv_data = (aht21_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.config.scl_port = dev->config.scl_port;
	priv_data->i2c.config.scl_pin = dev->config.scl_pin;
	priv_data->i2c.config.sda_port = dev->config.sda_port;
	priv_data->i2c.config.sda_pin = dev->config.sda_pin;
	
	/* 配置软件I2C */
	i2c_soft_init(&priv_data->i2c);

	dev->init_flag = true;

	/* AHT21硬件初始化 */
	AHT21_DELAY_MS(40);
	priv_data->i2c.read_reg(&priv_data->i2c, AHT21_ADDRESS, AHT21_GET_STATUS, &bits);
	if (!((bits >> 3) & 0x01))
	{
		priv_data->i2c.write_regs(&priv_data->i2c, AHT21_ADDRESS, AHT21_INIT, 2, datas_init);
		AHT21_DELAY_MS(10);
	}

    /* 函数指针赋值 */
	dev->get_data = __aht21_get_data;
	dev->deinit = __aht21_deinit;
    
	return 0;
}

/******************************************************************************
 * @brief	AHT21获取温湿度
 * @param	dev    	:   aht21_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __aht21_get_data(aht21_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	aht21_priv_data_t *priv_data = (aht21_priv_data_t *)dev->priv_data;
	
	uint8_t bits, timeout = 100;
	uint8_t datas_measure[2] = {0x33, 0x00};
	uint8_t datas[6] = {0};
	uint32_t temp_t, humi_t;

	/* 触发测量 */
	priv_data->i2c.write_regs(&priv_data->i2c, AHT21_ADDRESS, AHT21_MEASURE, 2, datas_measure);

	/* 等待测量完成 */
	while (timeout > 0)
	{
		priv_data->i2c.read_reg(&priv_data->i2c, AHT21_ADDRESS, AHT21_GET_STATUS, &bits);
		if ((bits & 0x80) == 0)
		{
			break;
		}

		timeout -= 10;
		AHT21_DELAY_MS(10);
	}
	if (timeout == 0)
	{
		return -2;
	}
	
	/* 读取数据 */
	priv_data->i2c.read_regs(&priv_data->i2c, AHT21_ADDRESS, AHT21_GET_STATUS + 1, 6, datas);

	/* 数据转换 */
	humi_t = (datas[1] << 12) + (datas[2] << 4) + (datas[3] >> 4);		// 拼接完成的20位湿度数据
	dev->data.humidity = (humi_t * 1000 >> 20);							// 湿度数据百分比乘10
	dev->data.humidity = dev->data.humidity / 10;						// 湿度数据百分比
	temp_t = ((datas[3] & 0x0F) << 16) + (datas[4] << 8) + datas[5];	// 拼接完成的20位温度数据
	dev->data.temperature = (temp_t * 2000 >> 20) - 500;				// 温度数据乘10
	dev->data.temperature = dev->data.temperature / 10;					// 温度数据

	return 0;
}

/******************************************************************************
 * @brief	去初始化AHT21
 * @param	dev   :  aht21_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __aht21_deinit(aht21_dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;

	aht21_priv_data_t *priv_data = (aht21_priv_data_t *)dev->priv_data;

	/* 去初始化软件I2C */
	priv_data->i2c.deinit(&priv_data->i2c);

	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志

    return 0;
}
