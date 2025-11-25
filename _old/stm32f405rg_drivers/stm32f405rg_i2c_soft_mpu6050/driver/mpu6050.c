#include "mpu6050.h"

/* MPU6050私有数据结构体 */
typedef struct {
	i2c_soft_dev_t i2c;	// 软件I2C设备
} mpu6050_priv_data_t;

/* 函数声明 */
static uint8_t __mpu6050_get_id(mpu6050_dev_t *dev);
static int8_t __mpu6050_get_data(mpu6050_dev_t *dev);
static int8_t __mpu6050_deinit(mpu6050_dev_t *dev);

/******************************************************************************
 * @brief	初始化MPU6050
 * @param	dev	:	mpu6050_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t mpu6050_init(mpu6050_dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 保存私有数据 */
	dev->priv_data = (mpu6050_priv_data_t *)malloc(sizeof(mpu6050_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	mpu6050_priv_data_t *priv_data = (mpu6050_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.config.scl_port = dev->config.scl_port;
	priv_data->i2c.config.scl_pin = dev->config.scl_pin;
	priv_data->i2c.config.sda_port = dev->config.sda_port;
	priv_data->i2c.config.sda_pin = dev->config.sda_pin;
	
	/* 配置软件I2C */
	i2c_soft_init(&priv_data->i2c);

	dev->init_flag = true;
	
	/* 函数指针赋值 */
	dev->get_id = __mpu6050_get_id;
	dev->get_data = __mpu6050_get_data;
	dev->deinit = __mpu6050_deinit;
	
	/* MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器 */
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01);		// 电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00);		// 电源管理寄存器2，保持默认值0，所有轴均不待机
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x09);		// 采样率分频寄存器，配置采样率
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);			// 配置寄存器，配置DLPF
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);		// 陀螺仪配置寄存器，选择满量程为±2000°/s
	priv_data->i2c.write_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x18);		// 加速度计配置寄存器，选择满量程为±16g
	
	return 0;
}

/******************************************************************************
 * @brief	MPU6050获取ID号
 * @param	dev	:	mpu6050_dev_t 结构体指针
 * @return	MPU6050的ID号
 ******************************************************************************/
static uint8_t __mpu6050_get_id(mpu6050_dev_t *dev)
{
	mpu6050_priv_data_t *priv_data = (mpu6050_priv_data_t *)dev->priv_data;
	uint8_t id;

	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_WHO_AM_I, &id);

	return id;
}

/******************************************************************************
 * @brief	MPU6050获取数据
 * @param	dev	:	mpu6050_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __mpu6050_get_data(mpu6050_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	mpu6050_priv_data_t *priv_data = (mpu6050_priv_data_t *)dev->priv_data;

	int16_t temp_val;
	uint8_t data_h, data_l;									    // 定义数据高8位和低8位的变量
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, &data_h);	// 读取加速度计X轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_L, &data_l);	// 读取加速度计X轴的低8位数据
	dev->data.accx = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_H, &data_h);	// 读取加速度计Y轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_L, &data_l);	// 读取加速度计Y轴的低8位数据
	dev->data.accy = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H, &data_h);	// 读取加速度计Z轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_L, &data_l);	// 读取加速度计Z轴的低8位数据
	dev->data.accz = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, &data_h);	// 读取陀螺仪X轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_L, &data_l);	// 读取陀螺仪X轴的低8位数据
	dev->data.gyrox = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_YOUT_H, &data_h);	// 读取陀螺仪Y轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_YOUT_L, &data_l);	// 读取陀螺仪Y轴的低8位数据
	dev->data.gyroy = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_H, &data_h);	// 读取陀螺仪Z轴的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_L, &data_l);	// 读取陀螺仪Z轴的低8位数据
	dev->data.gyroz = (data_h << 8) | data_l;					// 数据拼接
	
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_TEMP_OUT_H, &data_h);		// 读取温度值的高8位数据
	priv_data->i2c.read_reg(&priv_data->i2c, MPU6050_ADDRESS, MPU6050_TEMP_OUT_L, &data_l);		// 读取温度值的低8位数据
	temp_val = ((data_h << 8) | data_l);					    // 数据拼接
	dev->data.temp = (float)(temp_val)/340.0f + 36.53f;			// 计算温度值

	return 0;
}

/******************************************************************************
 * @brief	去初始化MPU6050
 * @param	dev	:  mpu6050_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __mpu6050_deinit(mpu6050_dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
    
    mpu6050_priv_data_t *priv_data = (mpu6050_priv_data_t *)dev->priv_data;
	
	/* 去初始化软件I2C */
	priv_data->i2c.deinit(&priv_data->i2c);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
    
    return 0;
}
