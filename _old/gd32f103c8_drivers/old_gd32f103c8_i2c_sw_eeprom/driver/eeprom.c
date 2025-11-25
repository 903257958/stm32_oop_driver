#include "eeprom.h"

/* EEPROM私有数据结构体 */
typedef struct {
	i2c_soft_dev_t i2c;		// I2C设备
} eeprom_priv_data_t;	

/* 函数声明 */
static int8_t __eeprom_write_byte(eeprom_dev_t *dev, uint8_t addr, uint8_t data);
static int8_t __eeprom_write_page(eeprom_dev_t *dev, uint8_t addr, uint8_t *data);
static int8_t __eeprom_read_data(eeprom_dev_t *dev, uint8_t addr, uint16_t num, uint8_t *data);
static int8_t __eeprom_deinit(eeprom_dev_t *dev);

/******************************************************************************
 * @brief	初始化EEPROM
 * @param	dev	:	eeprom_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t eeprom_init(eeprom_dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 保存私有数据 */
	dev->priv_data = (eeprom_priv_data_t *)malloc(sizeof(eeprom_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	eeprom_priv_data_t *priv_data = (eeprom_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.config.scl_port = dev->config.scl_port;
	priv_data->i2c.config.scl_pin = dev->config.scl_pin;
	priv_data->i2c.config.sda_port = dev->config.sda_port;
	priv_data->i2c.config.sda_pin = dev->config.sda_pin;
	
	/* 配置I2C */
	i2c_soft_init(&priv_data->i2c);
	
	/* 函数指针赋值 */
	dev->write_byte = __eeprom_write_byte;
	dev->write_page = __eeprom_write_page;
	dev->read_data = __eeprom_read_data;
	dev->deinit = __eeprom_deinit;
	
	dev->init_flag = true;

	return 0;
}

/******************************************************************************
 * @brief	EEPROM写入单字节数据
 * @param	dev		:  eeprom_dev_t 结构体指针
 * @param	addr	:  写入的地址
 * @param	data	:  要写入的数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __eeprom_write_byte(eeprom_dev_t *dev, uint8_t addr, uint8_t data)
{
	if (!dev || !dev->init_flag)
        return -1;
	
	eeprom_priv_data_t *priv_data = (eeprom_priv_data_t *)dev->priv_data;

	priv_data->i2c.write_reg(&priv_data->i2c, EEPROM_ADDRESS, addr, data);

	return 0;
}

/******************************************************************************
 * @brief	EEPROM按页写入数据
 * @param	dev		:  eeprom_dev_t 结构体指针
 * @param	addr	:  写入的起始地址
 * @param	data	:  要写入的数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __eeprom_write_page(eeprom_dev_t *dev, uint8_t addr, uint8_t *data)
{
	if (!dev || !dev->init_flag)
        return -1;
	
	eeprom_priv_data_t *priv_data = (eeprom_priv_data_t *)dev->priv_data;

	priv_data->i2c.write_regs(&priv_data->i2c, EEPROM_ADDRESS, addr, EEPROM_PAGE_SIZE, data);

	return 0;
}

/******************************************************************************
 * @brief	EEPROM读取数据
 * @param	dev		:  eeprom_dev_t 结构体指针
 * @param	addr	:  读取的起始地址
 * @param	num		:  要读取数据的个数
 * @param	data	:  要读取的数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __eeprom_read_data(eeprom_dev_t *dev, uint8_t addr, uint16_t num, uint8_t *data)
{
	if (!dev || !dev->init_flag)
        return -1;
	
	eeprom_priv_data_t *priv_data = (eeprom_priv_data_t *)dev->priv_data;

	priv_data->i2c.read_regs(&priv_data->i2c, EEPROM_ADDRESS, addr, num, data);

	return 0;
}

/******************************************************************************
 * @brief	去初始化EEPROM
 * @param	dev   :  eeprom_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __eeprom_deinit(eeprom_dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
    
    eeprom_priv_data_t *priv_data = (eeprom_priv_data_t *)dev->priv_data;
	
	/* 去初始化软件I2C */
	priv_data->i2c.deinit(&priv_data->i2c);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	//修改初始化标志
    
    return 0;
}
