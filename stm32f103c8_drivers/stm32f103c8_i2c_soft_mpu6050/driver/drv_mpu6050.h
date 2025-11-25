#ifndef DRV_MPU6050_H
#define DRV_MPU6050_H

#include <stdint.h>
#include <stdbool.h>

/* MPU6050 的 I2C 从机地址 */
#ifndef MPU6050_ADDRESS
#define MPU6050_ADDRESS	0x68
#endif

/* MPU6050 寄存器 */
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75

typedef struct {
	int16_t accx;	// 加速度计X轴的数据，范围：-32768~32767
	int16_t accy;	// 加速度计Y轴的数据，范围：-32768~32767
	int16_t accz;	// 加速度计Z轴的数据，范围：-32768~32767
	int16_t gyrox;	// 陀螺仪计X轴的数据，范围：-32768~32767
	int16_t gyroy;	// 陀螺仪计Y轴的数据，范围：-32768~32767
	int16_t gyroz;	// 陀螺仪计Z轴的数据，范围：-32768~32767
	float temp;		// 温度值
} mpu6050_data_t;

/* I2C 操作接口结构体 */
typedef struct {
	int (*write_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
	int (*read_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
} mpu6050_i2c_ops_t;

/* 配置结构体 */
typedef struct {
	const mpu6050_i2c_ops_t *i2c_ops;
} mpu6050_cfg_t;

typedef struct mpu6050_dev mpu6050_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*get_id)(mpu6050_dev_t *dev, uint8_t *id);
	int (*get_data)(mpu6050_dev_t *dev, mpu6050_data_t *data);
	int (*deinit)(mpu6050_dev_t *dev);
} mpu6050_ops_t;

/* 设备结构体 */
struct mpu6050_dev {
	mpu6050_cfg_t cfg;
	const mpu6050_ops_t *ops;
};

/**
 * @brief   初始化 MPU6050 驱动
 * @param[out] dev mpu6050_dev_t 结构体指针
 * @param[in]  cfg mpu6050_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_mpu6050_init(mpu6050_dev_t *dev, const mpu6050_cfg_t *cfg);

#endif
