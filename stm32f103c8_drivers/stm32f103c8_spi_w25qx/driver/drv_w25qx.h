#ifndef DRV_W25QX_H
#define DRV_W25QX_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_W25QX_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_W25QX_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_W25QX_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t	gpio_port_t;
typedef uint32_t	gpio_pin_t;

#else
#error drv_w25qx.h: No processor defined!
#endif

#ifndef ETIMEDOUT 
#define ETIMEDOUT	7
#endif

/* W25QX 存储结构宏定义 */
#define W25QX_PAGE_SIZE         	256  							/* 每页256字节 */
#define W25QX_BLOCK_64KB_PAGE_CNT	(64 * 1024 / W25QX_PAGE_SIZE)	/* 每块包含256页 */
#define W25QX_SECTOR_4KB_PAGE_CNT   (4 * 1024 / W25QX_PAGE_SIZE)	/* 每扇区包含16页 */

/* W25QX 寄存器 */
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

/* SPI 操作接口结构体 */
typedef struct {
	int (*start)(gpio_port_t cs_port, gpio_pin_t cs_pin);
	int (*swap_byte)(uint8_t send, uint8_t *recv);
	int (*stop)(gpio_port_t cs_port, gpio_pin_t cs_pin);
} w25qx_spi_ops_t;

/* 配置结构体 */
typedef struct {
	const w25qx_spi_ops_t *spi_ops;
	gpio_port_t 		   cs_port;
	gpio_pin_t  		   cs_pin;
} w25qx_cfg_t;

typedef struct w25qx_dev w25qx_dev_t;

/* 操作接口结构体 */
typedef struct {
    int (*read_id)(w25qx_dev_t *dev, uint8_t *mid, uint16_t *did);
	int (*write_page)(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data);
	int (*write_data)(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data);
	int (*erase_sector_4kb)(w25qx_dev_t *dev, uint32_t addr);
	int (*erase_block_64kb)(w25qx_dev_t *dev, uint16_t index);
	int (*read_data)(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data);
	int (*wakeup)(w25qx_dev_t *dev);
	int (*deinit)(w25qx_dev_t *dev);
} w25qx_ops_t;

/* 设备结构体 */
struct w25qx_dev {
	w25qx_cfg_t cfg;
	const w25qx_ops_t *ops;
};

/**
 * @brief   初始化 ST7789V 驱动
 * @param[out] dev w25qx_dev_t 结构体指针
 * @param[in]  cfg w25qx_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_w25qx_init(w25qx_dev_t *dev, const w25qx_cfg_t *cfg);

#endif
