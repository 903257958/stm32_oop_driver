#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	
	typedef GPIO_TypeDef*	MPU6050_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	MPU6050_GPIO_Port;
	
#else
	#error mpu6050.h: No processor defined!
#endif

#ifndef mpu6050_log
	#define mpu6050_log(x) 
#endif

#define MPU6050_ADDRESS			0xD0	//MPU6050的I2C从机地址

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

#define MPU6050_INT_PIN_CFG		0x37
#define MPU6050_INT_ENABLE		0x38

typedef struct {
	MPU6050_GPIO_Port SCLPort;			// SCL端口
	uint32_t SCLPin;					// SCL引脚
	MPU6050_GPIO_Port SDAPort;			// SDA端口
	uint32_t SDAPin;					// SDA引脚
	MPU6050_GPIO_Port INTPort;			// INT端口
	uint32_t INTPin;					// INT引脚
	void (*irq_callback)(void);			// 中断回调函数
}MPU6050Info_t;

typedef struct {
	int16_t accX;						// 加速度计X轴的数据，范围：-32768~32767
	int16_t accY;						// 加速度计Y轴的数据，范围：-32768~32767
	int16_t accZ;						// 加速度计Z轴的数据，范围：-32768~32767
	int16_t gyroX;						// 陀螺仪计X轴的数据，范围：-32768~32767
	int16_t gyroY;						// 陀螺仪计Y轴的数据，范围：-32768~32767
	int16_t gyroZ;						// 陀螺仪计Z轴的数据，范围：-32768~32767
	float temp;							// 温度值
}MPU6050Data_t;

typedef struct MPU6050Dev {
	MPU6050Info_t info;
	MPU6050Data_t data;
	bool initFlag;														// 初始化标志
	void *pPrivData;													// 私有数据指针
	uint8_t (*get_id)(struct MPU6050Dev *pDev);							// MPU6050获取ID号
	void (*get_data)(struct MPU6050Dev *pDev, MPU6050Data_t *data);		// MPU6050获取数据
	int (*deinit)(struct MPU6050Dev *pDev);								// 去初始化
}MPU6050Dev_t;

int mpu6050_init(MPU6050Dev_t *pDev);

#endif
