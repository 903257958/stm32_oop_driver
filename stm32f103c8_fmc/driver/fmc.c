#include "fmc.h"

/* 函数声明 */
static int8_t __fmc_page_erase(FMCDev_t *dev, uint16_t index, uint16_t num);
static int8_t __fmc_write(FMCDev_t *dev, uint32_t addr, uint32_t *data, uint32_t num);
static int8_t __fmc_deinit(FMCDev_t *dev);

/******************************************************************************
 * @brief	初始化FMC
 * @param	dev	:	FMCDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t fmc_init(FMCDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 函数指针赋值 */
	dev->page_erase = __fmc_page_erase;
	dev->write = __fmc_write;
	dev->deinit = __fmc_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	FMC页擦除，对于GD32F10x_MD，Flash为64KB，闪存页大小为1KB，共64页
 * @param	dev		:  FMCDev_t 结构体指针
 * @param	index   :  从第几页开始擦除
 * @param	num		:  擦除页的数量
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __fmc_page_erase(FMCDev_t *dev, uint16_t index, uint16_t num)
{
	if (!dev || !dev->init_flag)
		return -1;

	uint16_t i;

	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	FLASH_Unlock();
	for(i = 0; i < num; i++)
	{
		FLASH_ErasePage((0x08000000 + index * 1024) + i * 1024);
	}
	FLASH_Lock();

	#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)

	fmc_unlock();
	for (i = 0; i < num; i++)
	{
		fmc_page_erase((0x08000000 + index * 1024) + i * 1024);	// Flash的起始地址为0x08000000
	}
	fmc_lock();

	#endif
	
	return 0;
}

/******************************************************************************
 * @brief	FMC写
 * @param	dev		:  FMCDev_t 结构体指针
 * @param	addr	:  起始地址
 * @param	data	:  要写入的数据（数组元素为字，4字节）
 * @param	num		:  写入数据的数量（单位：字节）
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __fmc_write(FMCDev_t *dev, uint32_t addr, uint32_t *data, uint32_t num)
{
	if (!dev || !dev->init_flag)
		return -1;

	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	FLASH_Unlock();	
	while(num)
	{
		FLASH_ProgramWord(addr, *data);
		num -= 4;
		addr += 4;
		data++;
	}
	FLASH_Lock();

	#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)

	fmc_unlock();
	while (num)
	{
		fmc_word_program(addr, *data);	// 一次写入4字节
		num -= 4;
		addr += 4;
		data++;
	}
	fmc_lock();

	#endif

	return 0;
}

/******************************************************************************
 * @brief	去初始化FMC
 * @param	dev   :  FMCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __fmc_deinit(FMCDev_t *dev)
{  
    if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	return 0;
}
