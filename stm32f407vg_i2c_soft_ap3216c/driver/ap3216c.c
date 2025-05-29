#include "ap3216c.h"

/* AP3216C私有数据结构体 */
typedef struct {
	i2c_soft_dev_t i2c;		// I2C设备
}ap3216c_priv_data_t;	

/* 函数声明 */
static int8_t __ap3216c_get_data(ap3216c_dev_t *dev);
static int8_t __ap3216c_deinit(ap3216c_dev_t *dev);

/******************************************************************************
 * @brief	初始化AP3216C
 * @param	dev	:	ap3216c_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t ap3216c_init(ap3216c_dev_t *dev)
{
	if (!dev)
		return -1;

	uint8_t data;
	
	/* 保存私有数据 */
	dev->priv_data = (ap3216c_priv_data_t *)malloc(sizeof(ap3216c_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	ap3216c_priv_data_t *priv_data = (ap3216c_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.config.scl_port = dev->config.scl_port;
	priv_data->i2c.config.scl_pin = dev->config.scl_pin;
	priv_data->i2c.config.sda_port = dev->config.sda_port;
	priv_data->i2c.config.sda_pin = dev->config.sda_pin;
	
	/* 配置I2C */
	i2c_soft_init(&priv_data->i2c);
	
	/* 函数指针赋值 */
	dev->get_data = __ap3216c_get_data;
	dev->deinit = __ap3216c_deinit;

	/* 初始化AP3216C */
	priv_data->i2c.write_reg(&priv_data->i2c, AP3216C_ADDRESS, 0x00, 0x04);	// 复位AP3216C
    AP3216C_DELAY_MS(50);	// 复位至少10ms
	priv_data->i2c.write_reg(&priv_data->i2c, AP3216C_ADDRESS, 0x00, 0x03);	// 开启ALS、PS+IR

    priv_data->i2c.read_reg(&priv_data->i2c, AP3216C_ADDRESS, 0x00, &data);
    if (data != 0X03)
	{
		return -2;
	}
	
	dev->init_flag = true;

	return 0;
}

/******************************************************************************
 * @brief	AP3216C获取数据
 * @param	dev	:  ap3216c_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ap3216c_get_data(ap3216c_dev_t *dev)
{
	if (!dev || !dev->init_flag)
        return -1;
	
	ap3216c_priv_data_t *priv_data = (ap3216c_priv_data_t *)dev->priv_data;
	uint8_t buf[6] = {0};
	uint8_t i;

    /* 读取6字节数据 */
	for (i = 0; i < 6; i++)
	{
		priv_data->i2c.read_reg(&priv_data->i2c, AP3216C_ADDRESS, 0X0A + i, &buf[i]);
	}

    /* 数据合并为16位 */
	if (buf[0] & 0x80)
	{
		dev->data.infrared = 0;		// IR_OF位为1，则数据无效
	}
	else
	{
		dev->data.infrared = ((uint16_t)buf[1] << 2) | (buf[0] & 0X03);
	}

	dev->data.light = ((uint16_t)buf[3] << 8) | buf[2];

	if (buf[4] & 0x40)
	{
		dev->data.proximity = 0;	// IR_OF位为1，则数据无效
	}
	else
	{
		dev->data.proximity = ((uint16_t)(buf[5] & 0X3F) << 4) | (buf[4] & 0X0F);
	}

	return 0;
}

/******************************************************************************
 * @brief	去初始化AP3216C
 * @param	dev	:  ap3216c_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ap3216c_deinit(ap3216c_dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
    
    ap3216c_priv_data_t *priv_data = (ap3216c_priv_data_t *)dev->priv_data;
	
	/* 去初始化软件I2C */
	priv_data->i2c.deinit(&priv_data->i2c);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	//修改初始化标志
    
    return 0;
}
