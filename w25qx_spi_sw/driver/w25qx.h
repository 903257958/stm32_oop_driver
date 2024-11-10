#ifndef __W25QX_H
#define __W25QX_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "spi.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
	
    typedef GPIO_TypeDef*	W25QX_GPIO_Port;
	
#elif defined(STM32F40_41xxx)
	#include "stm32f4xx.h"
	
	typedef GPIO_TypeDef*	W25QX_GPIO_Port;
	
#else
    #error w25qx.h: No processor defined!
#endif

#ifndef W25QX_Log
    #define W25QX_Log(x) 
#endif

#define W25QX_PAGE_SIZE				256
#define W25QX_PER_WRITE_PAGE_SIZE	256

//#define  SPI_FLASH_ID		0x3015	//W25QX16
//#define  SPI_FLASH_ID		0x4015	//W25Q16
#define  SPI_FLASH_ID		0X4018	//W25Q128
//#define  SPI_FLASH_ID		0X4017	//W25Q64

#define W25QX_WRITE_ENABLE						0x06
#define W25QX_WRITE_DISABLE						0x04
#define W25QX_READ_STATUS_REGISTER_1			0x05
#define W25QX_READ_STATUS_REGISTER_2			0x35
#define W25QX_WRITE_STATUS_REGISTER				0x01
#define W25QX_PAGE_PROGRAM						0x02
#define W25QX_QUAD_PAGE_PROGRAM					0x32
#define W25QX_BLOCK_ERASE_64KB					0xD8
#define W25QX_BLOCK_ERASE_32KB					0x52
#define W25QX_SECTOR_ERASE_4KB					0x20
#define W25QX_CHIP_ERASE						0xC7
#define W25QX_ERASE_SUSPEND						0x75
#define W25QX_ERASE_RESUME						0x7A
#define W25QX_POWER_DOWN						0xB9
#define W25QX_HIGH_PERFORMANCE_MODE				0xA3
#define W25QX_CONTINUOUS_READ_MODE_RESET		0xFF
#define W25QX_RELEASE_POWER_DOWN_HPM_DEVICE_ID	0xAB
#define W25QX_MANUFACTURER_DEVICE_ID			0x90
#define W25QX_READ_UNIQUE_ID					0x4B
#define W25QX_JEDEC_ID							0x9F
#define W25QX_READ_DATA							0x03
#define W25QX_FAST_READ							0x0B
#define W25QX_FAST_READ_DUAL_OUTPUT				0x3B
#define W25QX_FAST_READ_DUAL_IO					0xBB
#define W25QX_FAST_READ_QUAD_OUTPUT				0x6B
#define W25QX_FAST_READ_QUAD_IO					0xEB
#define W25QX_OCTAL_WORD_READ_QUAD_IO			0xE3
#define W25QX_DUMMY_BYTE						0xFF

typedef struct {
	W25QX_GPIO_Port SCKPort;	// SCK端口
	uint32_t SCKPin;			// SCK引脚
	W25QX_GPIO_Port MOSIPort;	// MOSI端口
	uint32_t MOSIPin;			// MOSI引脚
	W25QX_GPIO_Port MISOPort;	// MISO端口
	uint32_t MISOPin;			// MISO引脚
	W25QX_GPIO_Port CSPort;		// CS端口
	uint32_t CSPin;				// CS引脚
}W25QXInfo_t;

typedef struct W25QXDev {
	W25QXInfo_t info;
	bool initFlag;																						// 初始化标志
	void *pPrivData;																					// 私有数据指针
	void (*read_id)(struct W25QXDev *pDev, uint8_t *mid, uint16_t *did);								// W25QX读取ID号
	void (*page_program)(struct W25QXDev *pDev, uint32_t address, uint8_t *dataArray, uint16_t count);	// W25QX页编程
	void (*write_data)(struct W25QXDev *pDev, uint32_t address, uint8_t *dataArray, uint32_t count);	// W25QX写入不定量数据
	void (*sector_erase)(struct W25QXDev *pDev, uint32_t address);										// W25QX扇区擦除（4KB）
	void (*read_data)(struct W25QXDev *pDev, uint32_t address, uint8_t *dataArray, uint32_t count);		// W25QX读取数据
	void (*wakeup)(struct W25QXDev *pDev);																// 唤醒W25QX
	int (*deinit)(struct W25QXDev *pDev);																// 去初始化
}W25QXDev_t;

int w25qx_init(W25QXDev_t *pDev);

#endif
