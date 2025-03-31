#include "delay.h"
#include "aht21.h"

/* AHT21私有数据结构体 */
typedef struct {
	I2CDev_t i2c;	// 软件I2C设备
}AHT21PrivData_t;

/* 函数声明 */
static int __aht21_get_data(AHT21Dev_t *dev);
static int __aht21_deinit(AHT21Dev_t *dev);

/******************************************************************************
 * @brief	初始化AHT21
 * @param	dev	:  AHT21Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int aht21_init(AHT21Dev_t *dev)
{
    if (!dev)
		return -1;

	uint8_t bits;
	uint8_t datas_init[2] = {0x08, 0x00};

    /* 保存私有数据 */
	dev->priv_data = (AHT21PrivData_t *)malloc(sizeof(AHT21PrivData_t));
	if (!dev->priv_data)
		return -1;
	
	AHT21PrivData_t *priv_data = (AHT21PrivData_t *)dev->priv_data;
	
	priv_data->i2c.info.scl_port = dev->info.scl_port;
	priv_data->i2c.info.scl_pin = dev->info.scl_pin;
	priv_data->i2c.info.sda_port = dev->info.sda_port;
	priv_data->i2c.info.sda_pin = dev->info.sda_pin;
	
	/* 配置软件I2C */
	i2c_init(&priv_data->i2c);

	dev->init_flag = true;

	/* AHT21硬件初始化 */
	delay_ms(40);
	priv_data->i2c.read_reg(&priv_data->i2c, AHT21_ADDRESS, AHT21_GET_STATUS, &bits);
	if (!((bits >> 3) & 0x01))
	{
		priv_data->i2c.write_regs(&priv_data->i2c, AHT21_ADDRESS, AHT21_INIT, 2, datas_init);
		delay_ms(10);
	}

    /* 函数指针赋值 */
	dev->get_data = __aht21_get_data;
	dev->deinit = __aht21_deinit;
    
	return 0;
}

/******************************************************************************
 * @brief	AHT21获取温湿度
 * @param	dev    	:   AHT21Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __aht21_get_data(AHT21Dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	AHT21PrivData_t *priv_data = (AHT21PrivData_t *)dev->priv_data;
	
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
		delay_ms(10);
	}
	if (timeout == 0)
	{
		return -2;
	}
	
	/* 读取数据 */
	priv_data->i2c.read_regs(&priv_data->i2c, AHT21_ADDRESS, AHT21_GET_STATUS + 1, 6, datas);

	/* 数据转换 */
	humi_t = (datas[1] << 12) + (datas[2] << 4) + (datas[3] >> 4);		// 拼接完成的20位湿度数据
	dev->humi = (humi_t * 1000 >> 20);									// 湿度数据百分比乘10
	dev->humi = dev->humi / 10;											// 湿度数据百分比
	temp_t = ((datas[3] & 0x0F) << 16) + (datas[4] << 8) + datas[5];	// 拼接完成的20位温度数据
	dev->temp = (temp_t * 2000 >> 20) - 500;							// 温度数据乘10
	dev->temp = dev->temp / 10;											// 温度数据

	return 0;
}

/******************************************************************************
 * @brief	去初始化AHT21
 * @param	dev   :  AHT21Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __aht21_deinit(AHT21Dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志

    return 0;
}
