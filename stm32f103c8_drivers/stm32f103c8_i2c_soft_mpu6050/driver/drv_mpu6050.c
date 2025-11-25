#include "drv_mpu6050.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 核心驱动层 --------------------------------- */

static int mpu6050_get_id_impl(mpu6050_dev_t *dev, uint8_t *id);
static int mpu6050_get_data_impl(mpu6050_dev_t *dev, mpu6050_data_t *data);
static int mpu6050_deinit_impl(mpu6050_dev_t *dev);

/* 操作接口表 */
static const mpu6050_ops_t mpu6050_ops = {
	.get_id   = mpu6050_get_id_impl,
	.get_data = mpu6050_get_data_impl,
	.deinit   = mpu6050_deinit_impl
};

/**
 * @brief   初始化 MPU6050 驱动
 * @param[out] dev mpu6050_dev_t 结构体指针
 * @param[in]  cfg mpu6050_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_mpu6050_init(mpu6050_dev_t *dev, const mpu6050_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	dev->cfg = *cfg;
	dev->ops = &mpu6050_ops;

	/* MPU6050 寄存器初始化 */
	cfg->i2c_ops->write_reg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01);		// 电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	cfg->i2c_ops->write_reg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00);		// 电源管理寄存器2，保持默认值0，所有轴均不待机
	cfg->i2c_ops->write_reg(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x09);		// 采样率分频寄存器，配置采样率
	cfg->i2c_ops->write_reg(MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);			// 配置寄存器，配置DLPF
	cfg->i2c_ops->write_reg(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);	// 陀螺仪配置寄存器，选择满量程为±2000°/s
	cfg->i2c_ops->write_reg(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x18);	// 加速度计配置寄存器，选择满量程为±16g
	return 0;
}

/**
 * @brief   MPU6050 获取 ID
 * @param[in] dev mpu6050_dev_t 结构体指针
 * @param[in] id  ID
 * @return	0 表示成功，其他值表示失败
 */
static int mpu6050_get_id_impl(mpu6050_dev_t *dev, uint8_t *id)
{
	if (!dev)
		return -EINVAL;
	
	return dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_WHO_AM_I, id);
}

/**
 * @brief   MPU6050 获取数据
 * @param[in] dev  mpu6050_dev_t 结构体指针
 * @param[in] data MPU6050 数据
 * @return	0 表示成功，其他值表示失败
 */
static int mpu6050_get_data_impl(mpu6050_dev_t *dev, mpu6050_data_t *data)
{
	if (!dev)
		return -EINVAL;
	
	int16_t temp_val;
	uint8_t data_h, data_l;

	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, &data_h);
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_L, &data_l);
	data->accx = (data_h << 8) | data_l;

	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_H, &data_h);
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_L, &data_l);
	data->accy = (data_h << 8) | data_l;
	
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H, &data_h);
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_L, &data_l);
	data->accz = (data_h << 8) | data_l;
	
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, &data_h);
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_L, &data_l);
	data->gyrox = (data_h << 8) | data_l;
	
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_H, &data_h);
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_L, &data_l);
	data->gyroy = (data_h << 8) | data_l;
	
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_H, &data_h);
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_L, &data_l);
	data->gyroz = (data_h << 8) | data_l;
	
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_TEMP_OUT_H, &data_h);
	dev->cfg.i2c_ops->read_reg(MPU6050_ADDRESS, MPU6050_TEMP_OUT_L, &data_l);
	temp_val = ((data_h << 8) | data_l);
	data->temp = (float)(temp_val)/340.0f + 36.53f;

	return 0;
}

/**
 * @brief   去初始化 MPU6050 设备
 * @param[in] dev mpu6050_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int mpu6050_deinit_impl(mpu6050_dev_t *dev)
{    
    if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
