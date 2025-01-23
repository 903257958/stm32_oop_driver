#include <stdlib.h>
#include "mpu6050.h"

/* MPU6050私有数据结构体 */
typedef struct {
	I2CDev_t mpu6050;		// 软件I2C设备
}MPU6050PrivData_t;

/* 通信协议 */
static void __mpu6050_write_reg(MPU6050Dev_t *pDev, uint8_t regAddr, uint8_t data);
static uint8_t __mpu6050_read_reg(MPU6050Dev_t *pDev, uint8_t regAddr);

/* 功能函数 */
static uint8_t __mpu6050_get_id(MPU6050Dev_t *pDev);
static void __mpu6050_get_data(MPU6050Dev_t *pDev, MPU6050Data_t *data);
static int __mpu6050_deinit(MPU6050Dev_t *pDev);

/******************************************************************************
 * @brief	初始化MPU6050
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int mpu6050_init(MPU6050Dev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 保存私有数据 */
	pDev->pPrivData = (MPU6050PrivData_t *)malloc(sizeof(MPU6050PrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	MPU6050PrivData_t *pPrivData = (MPU6050PrivData_t *)pDev->pPrivData;
	
	pPrivData->mpu6050.info.SCLPort = pDev->info.SCLPort;
	pPrivData->mpu6050.info.SCLPin = pDev->info.SCLPin;
	pPrivData->mpu6050.info.SDAPort = pDev->info.SDAPort;
	pPrivData->mpu6050.info.SDAPin = pDev->info.SDAPin;
	
	/* 配置软件I2C */
	i2c_init(&pPrivData->mpu6050);
	
	/* 函数指针赋值 */
	pDev->get_id = __mpu6050_get_id;
	pDev->get_data = __mpu6050_get_data;
	pDev->deinit = __mpu6050_deinit;
	
	pDev->initFlag = true;
	
	/* MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器 */
	__mpu6050_write_reg(pDev, MPU6050_PWR_MGMT_1, 0x01);		// 电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	__mpu6050_write_reg(pDev, MPU6050_PWR_MGMT_2, 0x00);		// 电源管理寄存器2，保持默认值0，所有轴均不待机
	__mpu6050_write_reg(pDev, MPU6050_SMPLRT_DIV, 0x09);		// 采样率分频寄存器，配置采样率
	__mpu6050_write_reg(pDev, MPU6050_CONFIG, 0x06);			// 配置寄存器，配置DLPF
	__mpu6050_write_reg(pDev, MPU6050_GYRO_CONFIG, 0x18);		// 陀螺仪配置寄存器，选择满量程为±2000°/s
	__mpu6050_write_reg(pDev, MPU6050_ACCEL_CONFIG, 0x18);		// 加速度计配置寄存器，选择满量程为±16g
	
	return 0;
}

/******************************************************************************
 * @brief	MPU6050写寄存器
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @param	regAddr		:	寄存器地址，范围：参考MPU6050手册的寄存器描述
 * @param	data		:	要写入寄存器的数据，范围：0x00~0xFF
 * @return	无
 ******************************************************************************/
static void __mpu6050_write_reg(MPU6050Dev_t *pDev, uint8_t regAddr, uint8_t data)
{
	MPU6050PrivData_t *pPrivData = (MPU6050PrivData_t *)pDev->pPrivData;
	
	pPrivData->mpu6050.start(&pPrivData->mpu6050);						// I2C起始
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, MPU6050_ADDRESS);	// 发送从机地址，读写位为0，表示即将写入
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);					// 接收应答
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, regAddr);			// 发送寄存器地址
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);					// 接收应答
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, data);			// 发送要写入寄存器的数据
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);					// 接收应答
	pPrivData->mpu6050.stop(&pPrivData->mpu6050);						// I2C终止					
}

/******************************************************************************
 * @brief	MPU6050读寄存器
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @param	regAddr		:	寄存器地址，范围：参考MPU6050手册的寄存器描述
 * @return	读取寄存器的数据，范围：0x00~0xFF
 ******************************************************************************/
static uint8_t __mpu6050_read_reg(MPU6050Dev_t *pDev, uint8_t regAddr)
{
	MPU6050PrivData_t *pPrivData = (MPU6050PrivData_t *)pDev->pPrivData;
	
	uint8_t data;
	
	pPrivData->mpu6050.start(&pPrivData->mpu6050);								// I2C起始
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, MPU6050_ADDRESS);			// 发送从机地址，读写位为0，表示即将写入
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);							// 接收应答
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, regAddr);					// 发送寄存器地址
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);							// 接收应答
	
	pPrivData->mpu6050.start(&pPrivData->mpu6050);								// I2C重复起始
	pPrivData->mpu6050.send_byte(&pPrivData->mpu6050, MPU6050_ADDRESS | 0x01);	// 发送从机地址，读写位为1，表示即将读取
	pPrivData->mpu6050.recv_ack(&pPrivData->mpu6050);							// 接收应答
	data = pPrivData->mpu6050.recv_byte(&pPrivData->mpu6050);					// 接收指定寄存器的数据
	pPrivData->mpu6050.send_ack(&pPrivData->mpu6050, 1);						// 发送应答，给从机非应答，终止从机的数据输出
	pPrivData->mpu6050.stop(&pPrivData->mpu6050);								// I2C终止					
	
	return data;
}

/******************************************************************************
 * @brief	MPU6050获取ID号
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @return	MPU6050的ID号
 ******************************************************************************/
static uint8_t __mpu6050_get_id(MPU6050Dev_t *pDev)
{
	return __mpu6050_read_reg(pDev, MPU6050_WHO_AM_I);		// 返回WHO_AM_I寄存器的值
}

/******************************************************************************
 * @brief	MPU6050获取数据
 * @param	pDev		:	MPU6050Dev_t 结构体指针
 * @param	data		:	MPU6050Data_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __mpu6050_get_data(MPU6050Dev_t *pDev, MPU6050Data_t *data)
{
	int16_t tempVal;
	uint8_t dataH, dataL;									// 定义数据高8位和低8位的变量
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_ACCEL_XOUT_H);	// 读取加速度计X轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_ACCEL_XOUT_L);	// 读取加速度计X轴的低8位数据
	data->accX = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_ACCEL_YOUT_H);	// 读取加速度计Y轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_ACCEL_YOUT_L);	// 读取加速度计Y轴的低8位数据
	data->accY = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_ACCEL_ZOUT_H);	// 读取加速度计Z轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_ACCEL_ZOUT_L);	// 读取加速度计Z轴的低8位数据
	data->accZ = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_GYRO_XOUT_H);	// 读取陀螺仪X轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_GYRO_XOUT_L);	// 读取陀螺仪X轴的低8位数据
	data->gyroX = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_GYRO_YOUT_H);	// 读取陀螺仪Y轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_GYRO_YOUT_L);	// 读取陀螺仪Y轴的低8位数据
	data->gyroY = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_GYRO_ZOUT_H);	// 读取陀螺仪Z轴的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_GYRO_ZOUT_L);	// 读取陀螺仪Z轴的低8位数据
	data->gyroZ = (dataH << 8) | dataL;						// 数据拼接
	
	dataH = __mpu6050_read_reg(pDev, MPU6050_TEMP_OUT_H);	// 读取温度值的高8位数据
	dataL = __mpu6050_read_reg(pDev, MPU6050_TEMP_OUT_L);	// 读取温度值的低8位数据
	tempVal = ((dataH << 8) | dataL);						// 数据拼接
	data->temp = (float)(tempVal)/340.0f + 36.53f;			// 计算温度值
}

/******************************************************************************
 * @brief	去初始化MPU6050
 * @param	pDev   :  MPU6050Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __mpu6050_deinit(MPU6050Dev_t *pDev)
{    
    if (!pDev || !pDev->initFlag)
        return -1;
    
    MPU6050PrivData_t *pPrivData = (MPU6050PrivData_t *)pDev->pPrivData;
	
	/* 去初始化软件I2C */
	pPrivData->mpu6050.deinit(&pPrivData->mpu6050);
	
	/* 释放私有数据内存 */
	free(pDev->pPrivData);
	pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	// 修改初始化标志
    
    return 0;
}
