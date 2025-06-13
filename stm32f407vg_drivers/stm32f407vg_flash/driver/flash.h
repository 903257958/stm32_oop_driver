#ifndef FLASH_DRV_H
#define FLASH_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
    #include "stm32f10x.h"
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"
    
#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)
    #include "gd32f10x.h"
	
#else
    #error flash.h: No processor defined!
#endif

typedef struct flash_dev {
	bool init_flag;		// 初始化标志
	void *priv_data;	// 私有数据指针
	int8_t (*page_erase)(struct flash_dev *dev, uint16_t index, uint16_t num);
	int8_t (*sector_erase)(struct flash_dev *dev, uint8_t index);
	int8_t (*write)(struct flash_dev *dev, uint32_t addr, uint32_t *data, uint32_t num);
	int8_t (*deinit)(struct flash_dev *dev);	// 去初始化
} flash_dev_t;

int8_t flash_init(flash_dev_t *dev);

#endif
