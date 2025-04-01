#ifndef __BMP280_H
#define __BMP280_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "i2c.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef GPIO_TypeDef*	BMP280GPIOPort_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
	typedef GPIO_TypeDef*	BMP280GPIOPort_t;
	
#else
	#error bmp280.h: No processor defined!
#endif

#ifndef bmp280_log
	#define bmp280_log(x)
#endif

/* BMP280的I2C从机地址 */
#define BMP280_ADDRESS			0x76

/* 复位寄存器写入值 */
#define BMP280_RESET_VALUE		0xB6

/* 芯片默认ID值 */
#define BMP280_DEFAULT_CHIP_ID	0x58

/* BMP280寄存器 */
#define BMP280_CHIP_ID									0xD0    // 芯片ID寄存器地址
#define BMP280_RESET_REG								0xE0    // 软件复位寄存器地址
#define BMP280_STAT_REG									0xF3    // 状态寄存器地址
#define BMP280_CTRL_MEAS_REG							0xF4    // 测量控制寄存器地址
#define BMP280_CONFIG_REG								0xF5    // 配置寄存器地址 
#define BMP280_PRESSURE_MSB_REG							0xF7    // 气压值最高有效位寄存器地址 
#define BMP280_PRESSURE_LSB_REG							0xF8    // 气压值最低有效位寄存器地址
#define BMP280_PRESSURE_XLSB_REG						0xF9    // 气压值扩展最低有效位寄存器地址
#define BMP280_TEMPERATURE_MSB_REG						0xFA    // 温度值最高有效位寄存器地址
#define BMP280_TEMPERATURE_LSB_REG						0xFB    // 温度值最低有效位寄存器地址
#define BMP280_TEMPERATURE_XLSB_REG						0xFC    // 温度值扩展最低有效位寄存器地址
#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG			0x88	// 温度校准参数T1的LSB寄存器地址

#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH	24		// 压力和温度校准数据长度(24字节)
#define BMP280_DATA_FRAME_SIZE							6		// 数据帧大小(6字节: 3字节压力 + 3字节温度) 

/* 工作模式定义 */
typedef enum {
	BMP280_SLEEP_MODE = 0x0,
	BMP280_FORCED_MODE = 0x1,
	BMP280_NORMAL_MODE = 0x3
}BMP280WorkMode_t;

/* 压力过采样因子 */
typedef enum {
	BMP280_P_MODE_SKIP = 0x0,	// skipped
	BMP280_P_MODE_1,			// x1
	BMP280_P_MODE_2,			// x2
	BMP280_P_MODE_3,			// x4
	BMP280_P_MODE_4,			// x8
	BMP280_P_MODE_5			    // x16
}BMP280PressureOversample_t;

/* 温度过采样因子 */
typedef enum {
	BMP280_T_MODE_SKIP = 0x0,	// skipped
	BMP280_T_MODE_1,			// x1
	BMP280_T_MODE_2,			// x2
	BMP280_T_MODE_3,			// x4
	BMP280_T_MODE_4,			// x8
	BMP280_T_MODE_5			    // x16
}BMP280TemperatureOversample_t;
									
/* IIR滤波器时间常数 */
typedef enum {
	BMP280_FILTER_OFF = 0x0,	// filter off
	BMP280_FILTER_MODE_1,		// 0.223*ODR x2
	BMP280_FILTER_MODE_2,		// 0.092*ODR x4
	BMP280_FILTER_MODE_3,		// 0.042*ODR x8
	BMP280_FILTER_MODE_4		// 0.021*ODR x16
}BMP280FliterCoefficient_t;

/* 保持时间 */
typedef enum {
	BMP280_TIME_STANDBY_1 = 0x0,	// 0.5ms
	BMP280_TIME_STANDBY_2,			// 62.5ms
	BMP280_TIME_STANDBY_3,			// 125ms
	BMP280_TIME_STANDBY_4,			// 250ms
	BMP280_TIME_STANDBY_5,			// 500ms
	BMP280_TIME_STANDBY_6,			// 1000ms
	BMP280_TIME_STANDBY_7,			// 2000ms
	BMP280_TIME_STANDBY_8,			// 4000ms
}BMPTimeStandby_t;

/* 设置过采样因子、保持时间和滤波器分频因子 */
typedef struct {
	BMP280PressureOversample_t 		pressure_oversample;
	BMP280TemperatureOversample_t 	temperature_oversample;
	BMP280WorkMode_t 				work_mode;
	BMPTimeStandby_t 				time_standby;
	BMP280FliterCoefficient_t 		filter_coefficient;
	bool							spi_en;
}BMP280Config_t;

typedef struct {
    BMP280GPIOPort_t scl_port;	// SCL端口
	uint32_t scl_pin;			// SCL引脚
	BMP280GPIOPort_t sda_port;	// SDA端口
	uint32_t sda_pin;			// SDA引脚
}BMP280Info_t;

typedef struct {
	float temperature;	// 温度值
	float pressure;		// 气压值
    float altitude;		// 海拔
}BMP280Data_t;

typedef struct BMP280Dev {
	BMP280Info_t info;
	BMP280Data_t data;
	bool init_flag;										// 初始化标志
    void *priv_data;									// 私有数据指针
	int (*get_id)(struct BMP280Dev *dev, uint8_t *id);	// 获取ID号
	int (*get_data)(struct BMP280Dev *dev);				// 获取数据
	int (*deinit)(struct BMP280Dev *dev); 				// 去初始化
}BMP280Dev_t;

int bmp280_init(BMP280Dev_t *dev);

#endif
