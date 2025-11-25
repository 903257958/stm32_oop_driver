#ifndef DRV_FLASH_H
#define DRV_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_FLASH_PLATFORM_STM32F1 1
#include "stm32f10x.h"
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_FLASH_PLATFORM_STM32F4 1
#include "stm32f4xx.h"
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
#define DRV_FLASH_PLATFORM_GD32F1 1
#include "gd32f10x.h"
	
#else
#error drv_flash.h: No processor defined!
#endif

#ifndef EIO
#define EIO 8
#endif

typedef struct flash_dev flash_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*page_erase)(flash_dev_t *dev, uint16_t cnt, uint16_t idx);
	int (*sector_erase)(flash_dev_t *dev, uint16_t cnt, uint8_t idx);
	int (*write)(flash_dev_t *dev, uint32_t addr, uint32_t cnt, uint32_t *data);
	int (*deinit)(flash_dev_t *dev);
} flash_ops_t;

/* 设备结构体 */
struct flash_dev {
	const flash_ops_t *ops;
};

/**
 * @brief   初始化内部 Flash 驱动
 * @param[out] dev flash_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_flash_init(flash_dev_t *dev);

#endif
