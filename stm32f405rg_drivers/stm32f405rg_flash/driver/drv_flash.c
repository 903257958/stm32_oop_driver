#include "drv_flash.h"
#include <stddef.h>
#include <errno.h>

static int flash_erase_page_impl(flash_dev_t *dev, uint16_t cnt, uint16_t idx);
static int flash_erase_sector_impl(flash_dev_t *dev, uint16_t cnt, uint8_t idx);
static int flash_write_impl(flash_dev_t *dev, uint32_t addr, uint32_t cnt, uint32_t *data);
static int flash_deinit_impl(flash_dev_t *dev);

/* 操作接口表 */
static const flash_ops_t flash_ops = {
	.page_erase   = flash_erase_page_impl,
	.sector_erase = flash_erase_sector_impl,
	.write        = flash_write_impl,
	.deinit       = flash_deinit_impl
};

/**
 * @brief   初始化内部 Flash 驱动
 * @param[out] dev flash_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_flash_init(flash_dev_t *dev)
{
	if (!dev)
        return -EINVAL;

	dev->ops = &flash_ops;
	return 0;
}


/**
 * @brief   内部 Flash 页擦除
 * @details 以 GD32F10x_MD 为例，Flash 为 64KB，闪存页大小为 1KB，共 64 页
 * @param[in] dev w25qx_dev_t 结构体指针
 * @param[in] cnt 擦除页的数量
 * @param[in] idx 擦除页的起始索引
 * @return	0 表示成功，其他值表示失败
 */
static int flash_erase_page_impl(flash_dev_t *dev, uint16_t cnt, uint16_t idx)
{
#if DRV_FLASH_PLATFORM_STM32F1 || DRV_FLASH_PLATFORM_GD32F1
	uint16_t i;
    uint32_t addr;

    (void)dev;

#if DRV_FLASH_PLATFORM_STM32F1
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY      |
                    FLASH_FLAG_PGERR    |
                    FLASH_FLAG_WRPRTERR);
	for (i = 0; i < cnt; i++) {
		addr = 0x08000000 + (idx + i) * 1024;
		if (FLASH_ErasePage(addr) != FLASH_COMPLETE) {
            FLASH_Lock();
            return -EIO;
        }
	}
	FLASH_Lock();

#elif DRV_FLASH_PLATFORM_GD32F1
	fmc_unlock();
	for (i = 0; i < cnt; i++) {
		addr = 0x08000000 + (idx + i) * 1024;
		if (fmc_page_erase(addr) != FMC_READY) {
            fmc_lock();
            return -EIO;
        }
	}
	fmc_lock();

#endif
	return 0;

#else
	return -EINVAL;
#endif
}

/**
 * @brief   内部 Flash 扇区擦除
 * @details 以 STM32F407VG 为例，Flash 为 1024KB，有 0~11 共 12 个扇区，扇区大小不统一
 * @param[in] dev w25qx_dev_t 结构体指针
 * @param[in] cnt 擦除扇区的数量
 * @param[in] idx 擦除扇区的起始索引
 * @return	0 表示成功，其他值表示失败
 */
static int flash_erase_sector_impl(flash_dev_t *dev, uint16_t cnt, uint8_t idx)
{
#if DRV_FLASH_PLATFORM_STM32F4
	uint8_t i;
    FLASH_Status status;
	uint32_t sector;
	static const uint32_t sector_tbl[] = {
		FLASH_Sector_0, FLASH_Sector_1, FLASH_Sector_2, FLASH_Sector_3,
		FLASH_Sector_4, FLASH_Sector_5, FLASH_Sector_6, FLASH_Sector_7,
		FLASH_Sector_8, FLASH_Sector_9, FLASH_Sector_10, FLASH_Sector_11
	};

    (void)dev;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR  |
                    FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                    FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    for (i = 0; i < cnt; i++) {
		sector = sector_tbl[idx + i];
        status = FLASH_EraseSector(sector, VoltageRange_3);
        if (status != FLASH_COMPLETE) {
            FLASH_Lock();
            return -EIO;
        }
    }

    FLASH_Lock();
    return 0;
#else
	return -EINVAL;
#endif
}

/**
 * @brief   写内部 Flash
 * @param[in] dev  w25qx_dev_t 结构体指针
 * @param[in] addr 起始地址
 * @param[in] cnt  写入数据的数量（单位：字节）
 * @param[in] data 要写入的数据（数组元素为字，4字节）
 * @return	0 表示成功，其他值表示失败
 */
static int flash_write_impl(flash_dev_t *dev, uint32_t addr, uint32_t cnt, uint32_t *data)
{
    uint32_t i;
    (void)dev;

    /* 地址必须 4 字节对齐 */
    if (addr & 0x3)
        return -EINVAL;

    /* 数据长度必须为 4 的倍数 */
    if (cnt & 0x3)
        return -EINVAL;

#if DRV_FLASH_PLATFORM_STM32F1
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY |
                    FLASH_FLAG_EOP |
                    FLASH_FLAG_PGERR |
                    FLASH_FLAG_WRPRTERR);

    for (i = 0; i < cnt; i += 4) {
        FLASH_Status st = FLASH_ProgramWord(addr + i, data[i / 4]);
        if (st != FLASH_COMPLETE) {
            FLASH_Lock();
            return -EIO; /* 写入失败 */
        }
    }
    FLASH_Lock();

#elif DRV_FLASH_PLATFORM_STM32F4
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP     | FLASH_FLAG_OPERR |
                    FLASH_FLAG_WRPERR  | FLASH_FLAG_PGAERR |
                    FLASH_FLAG_PGPERR  | FLASH_FLAG_PGSERR);

    for (i = 0; i < cnt; i += 4) {
        FLASH_Status st = FLASH_ProgramWord(addr + i, data[i / 4]);
        if (st != FLASH_COMPLETE) {
            FLASH_Lock();
            return -EIO;
        }
    }
    FLASH_Lock();

#elif DRV_FLASH_PLATFORM_GD32F1
    fmc_unlock();

    for (i = 0; i < cnt; i += 4) {
        fmc_status_ecnt st = fmc_word_program(addr + i, data[i / 4]);
        if (st != FMC_READY) {
            fmc_lock();
            return -EIO;
        }
    }

    fmc_lock();
#endif

    return 0;
}

/**
 * @brief   去初始化内部 Flash
 * @param[in] dev flash_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int flash_deinit_impl(flash_dev_t *dev)
{  
    if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}
