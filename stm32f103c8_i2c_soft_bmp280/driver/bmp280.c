#include "bmp280.h"

/* BMP280私有数据结构体 */
typedef struct {
	i2c_soft_dev_t i2c;	// 软件I2C设备
} bmp280_priv_data_t;

/* BMP280的24字节校准数据 */
typedef struct {
    uint16_t dig_t1;		/* calibration T1 data */
    int16_t dig_t2;			/* calibration T2 data */
    int16_t dig_t3;			/* calibration T3 data */
    uint16_t dig_p1;		/* calibration P1 data */
    int16_t dig_p2;			/* calibration P2 data */
    int16_t dig_p3;			/* calibration P3 data */
    int16_t dig_p4;			/* calibration P4 data */
    int16_t dig_p5;			/* calibration P5 data */
    int16_t dig_p6;			/* calibration P6 data */
    int16_t dig_p7;			/* calibration P7 data */
    int16_t dig_p8;			/* calibration P8 data */
    int16_t dig_p9;			/* calibration P9 data */
    int32_t t_fine;			/* calibration t_fine data */
}bmp280_calibration_data_t;

bmp280_calibration_data_t calibration_data;

/* 函数声明 */
static int8_t __bmp280_settings(bmp280_dev_t *dev, bmp280_setting_t *setting);
static int8_t __bmp280_get_id(bmp280_dev_t *dev, uint8_t *id);
static int8_t __bmp280_get_raw_data(bmp280_dev_t *dev, int32_t *pressure_raw, int32_t *temperature_raw);
static int8_t __bmp280_temperature_compensate(bmp280_dev_t *dev, int32_t *temperature_raw);
static int8_t __bmp280_pressure_compensate(bmp280_dev_t *dev, int32_t *pressure_raw);
static int8_t __bmp280_get_data(bmp280_dev_t *dev);
static int8_t __bmp280_deinit(bmp280_dev_t *dev);

/******************************************************************************
 * @brief	初始化BMP280
 * @param	dev	:  bmp280_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t bmp280_init(bmp280_dev_t *dev)
{
    if (!dev)
		return -1;

    /* 保存私有数据 */
	dev->priv_data = (bmp280_priv_data_t *)malloc(sizeof(bmp280_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	bmp280_priv_data_t *priv_data = (bmp280_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.config.scl_port = dev->config.scl_port;
	priv_data->i2c.config.scl_pin = dev->config.scl_pin;
	priv_data->i2c.config.sda_port = dev->config.sda_port;
	priv_data->i2c.config.sda_pin = dev->config.sda_pin;
	
	/* 配置软件I2C */
	i2c_soft_init(&priv_data->i2c);

	/* 函数指针赋值 */
	dev->get_id = __bmp280_get_id;
	dev->get_data = __bmp280_get_data;
	dev->deinit = __bmp280_deinit;

	dev->init_flag = true;

	/* BMP280硬件初始化 */
	/* 读取24字节校准数据 */
	priv_data->i2c.read_regs(&priv_data->i2c, BMP280_ADDRESS, 
							BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, (uint8_t *)&calibration_data);

	/* 往复位寄存器写入给定值 */
	priv_data->i2c.write_reg(&priv_data->i2c, BMP280_ADDRESS, BMP280_RESET_REG, BMP280_RESET_VALUE);
	BMP280_DELAY_MS(50);
	
	/* 设置过采样因子、保持时间和滤波器分频因子 */
	bmp280_setting_t	bmp280_setting_structure;
	bmp280_setting_structure.pressure_oversample = BMP280_P_MODE_3;
	bmp280_setting_structure.temperature_oversample = BMP280_T_MODE_1;
	bmp280_setting_structure.work_mode = BMP280_NORMAL_MODE;
	bmp280_setting_structure.time_standby = BMP280_TIME_STANDBY_1;
	bmp280_setting_structure.filter_coefficient = BMP280_FILTER_MODE_4;
	bmp280_setting_structure.spi_en = false;
	__bmp280_settings(dev, &bmp280_setting_structure);
	
	return 0;
}

/******************************************************************************
 * @brief	BMP280配置的测量参数和运行模式（过采样、待机时间、滤波器等）
 * @param	dev		:  bmp280_dev_t 结构体指针
 * @param	setting	:  bmp280_setting_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __bmp280_settings(bmp280_dev_t *dev, bmp280_setting_t *setting)
{
	if (!dev || !dev->init_flag)
        return -1;
	
	bmp280_priv_data_t *priv_data = (bmp280_priv_data_t *)dev->priv_data;

	uint8_t reg1, reg2;

	reg1 = 	((setting->temperature_oversample) << 5) | 
			((setting->pressure_oversample) << 2) | 
			((setting)->work_mode);

	reg2 = 	((setting->time_standby) << 5) | 
			((setting->filter_coefficient) << 2) | 
			((setting->spi_en));
	
	priv_data->i2c.write_reg(&priv_data->i2c, BMP280_ADDRESS, BMP280_CTRL_MEAS_REG, reg1);
	priv_data->i2c.write_reg(&priv_data->i2c, BMP280_ADDRESS, BMP280_CONFIG_REG, reg2);

	return 0;
}

/******************************************************************************
 * @brief	BMP280获取ID号
 * @param	dev	:	bmp280_dev_t 结构体指针
 * @param	id	:	BMP280的ID号
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __bmp280_get_id(bmp280_dev_t *dev, uint8_t *id)
{
	if (!dev || !dev->init_flag)
        return -1;
	
	bmp280_priv_data_t *priv_data = (bmp280_priv_data_t *)dev->priv_data;

	priv_data->i2c.read_reg(&priv_data->i2c, BMP280_ADDRESS, BMP280_CHIP_ID, id);

	return 0;
}

/******************************************************************************
 * @brief	BMP280获取传感器原始数据
 * @param	dev    			:   bmp280_dev_t 结构体指针
 * @param	pressure_raw	:   原始气压数据
 * @param	temperature_raw	:   原始温度数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __bmp280_get_raw_data(bmp280_dev_t *dev, int32_t *pressure_raw, int32_t *temperature_raw)
{
	if (!dev || !dev->init_flag)
        return -1;

	bmp280_priv_data_t *priv_data = (bmp280_priv_data_t *)dev->priv_data;

	uint8_t data[BMP280_DATA_FRAME_SIZE];
	
	priv_data->i2c.read_regs(&priv_data->i2c, BMP280_ADDRESS, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);
	*pressure_raw = (int32_t)((((uint32_t)data[0]) << 12) | (((uint32_t)data[1]) << 4) | ((uint32_t)data[2] >> 4));
	*temperature_raw = (int32_t)((((uint32_t)data[3]) << 12) | (((uint32_t)data[4]) << 4) | ((uint32_t)data[5] >> 4));

	return 0;
}

/******************************************************************************
 * @brief	BMP280温度值补偿
 * @param	dev    			:   bmp280_dev_t 结构体指针
 * @param	temperature_raw	:   原始温度数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __bmp280_temperature_compensate(bmp280_dev_t *dev, int32_t *temperature_raw)
{
	if (!dev || !dev->init_flag)
        return -1;

	double var1, var2;
	
	var1 = (((double)*temperature_raw) / 16384.0 - ((double)calibration_data.dig_t1) / 1024.0) * ((double)calibration_data.dig_t2);
	var2 = 	((((double)*temperature_raw) / 131072.0 - ((double)calibration_data.dig_t1) / 8192.0) *
			(((double)*temperature_raw) / 131072.0 - ((double) calibration_data.dig_t1) / 8192.0)) * 
			((double)calibration_data.dig_t3);
	calibration_data.t_fine = (int32_t)(var1 + var2);
	dev->data.temperature = (var1 + var2) / 5120.0;

	return 0;
}

/******************************************************************************
 * @brief	BMP280气压值补偿
 * @param	dev    			:   bmp280_dev_t 结构体指针
 * @param	pressure_raw	:   原始气压数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __bmp280_pressure_compensate(bmp280_dev_t *dev, int32_t *pressure_raw)
{
	if (!dev || !dev->init_flag)
        return -1;

	double var1, var2, p;
	
	var1 = ((double)calibration_data.t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calibration_data.dig_p6) / 32768.0;
	var2 = var2 + var1 * ((double)calibration_data.dig_p5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calibration_data.dig_p4) * 65536.0);
	var1 = (((double)calibration_data.dig_p3) * var1 * var1 / 524288.0 + ((double)calibration_data.dig_p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)calibration_data.dig_p1);
	if (var1 == 0.0)
	{
		dev->data.pressure = 0;
		return -2;
	}
	p = 1048576.0 - (double)(*pressure_raw);
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)calibration_data.dig_p9) * p * p / 2147483648.0;
	var2 = p * ((double)calibration_data.dig_p8) / 32768.0;
	p = p + (var1 + var2 + ((double)calibration_data.dig_p7)) / 16.0;
	dev->data.pressure = p / 100.0;

	return 0;
}

/******************************************************************************
 * @brief	BMP280获取数据
 * @param	dev	:   bmp280_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __bmp280_get_data(bmp280_dev_t *dev)
{
	if (!dev || !dev->init_flag)
        return -1;

	int32_t pressure_raw = 0;
	int32_t temperature_raw = 0;
	
	/* 获取传感器数据 */
	__bmp280_get_raw_data(dev, &pressure_raw, &temperature_raw);

	/* 数据补偿并保存 */
	__bmp280_temperature_compensate(dev, &temperature_raw);
	__bmp280_pressure_compensate(dev, &pressure_raw);

	return 0;
}

/******************************************************************************
 * @brief	去初始化BMP280
 * @param	dev   :  bmp280_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __bmp280_deinit(bmp280_dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;

	bmp280_priv_data_t *priv_data = (bmp280_priv_data_t *)dev->priv_data;
	
	/* 去初始化软件I2C */
	priv_data->i2c.deinit(&priv_data->i2c);

	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志

    return 0;
}
