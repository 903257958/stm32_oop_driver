#ifndef MPU6050_DRV_H
#define MPU6050_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	
#else
	#error mpu6050.h: No processor defined!
#endif

#endif

#include "i2c_soft.h"

/* MPU6050的I2C从机地址 */
#define MPU6050_ADDRESS			0x68

/* MAX30102寄存器 */
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
	gpio_port_t scl_port;	// SCL端口
	gpio_pin_t scl_pin;		// SCL引脚
	gpio_port_t sda_port;	// SDA端口
	gpio_pin_t sda_pin;		// SDA引脚
} mpu6050_config_t;

typedef struct {
	int16_t accx;	// 加速度计X轴的数据，范围：-32768~32767
	int16_t accy;	// 加速度计Y轴的数据，范围：-32768~32767
	int16_t accz;	// 加速度计Z轴的数据，范围：-32768~32767
	int16_t gyrox;	// 陀螺仪计X轴的数据，范围：-32768~32767
	int16_t gyroy;	// 陀螺仪计Y轴的数据，范围：-32768~32767
	int16_t gyroz;	// 陀螺仪计Z轴的数据，范围：-32768~32767
	float temp;		// 温度值
} mpu6050_data_t;

typedef struct mpu6050_dev {
	mpu6050_config_t config;
	mpu6050_data_t data;
	bool init_flag;									// 初始化标志
	void *priv_data;								// 私有数据指针
	uint8_t (*get_id)(struct mpu6050_dev *dev);		// MPU6050获取ID号
	int8_t (*get_data)(struct mpu6050_dev *dev); 	// MPU6050获取数据
	int8_t (*deinit)(struct mpu6050_dev *dev);		// 去初始化
} mpu6050_dev_t;

int8_t mpu6050_init(mpu6050_dev_t *dev);

#endif
