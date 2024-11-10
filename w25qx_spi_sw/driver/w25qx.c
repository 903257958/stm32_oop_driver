#include "w25qx.h"

/* W25QXX私有数据结构体 */
typedef struct {
	SPIDev_t w25qx;		// 软件SPI设备
}W25QXPrivData_t;

/* 函数声明 */
static void __w25qx_read_id(W25QXDev_t *pDev, uint8_t *mid, uint16_t *did);
static void __w25qx_page_program(W25QXDev_t *pDev, uint32_t address, uint8_t *dataArray, uint16_t count);
static void __w25qx_write_data(W25QXDev_t *pDev, uint32_t address, uint8_t *dataArray, uint32_t count);
static void __w25qx_sector_erase(W25QXDev_t *pDev, uint32_t address);
static void __w25qx_read_data(W25QXDev_t *pDev, uint32_t address, uint8_t *dataArray, uint32_t count);
static void __w25qx_wake_up(W25QXDev_t *pDev);
static int __w25qx_deinit(W25QXDev_t *pDev);

/******************************************************************************
 * @brief	初始化W25QX
 * @param	pDev	:	W25QXDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int w25qx_init(W25QXDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 初始化私有数据 */
	pDev->pPrivData = (W25QXPrivData_t *)malloc(sizeof(W25QXPrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	pPrivData->w25qx.info.SCKPort = pDev->info.SCKPort;
	pPrivData->w25qx.info.SCKPin = pDev->info.SCKPin;
	pPrivData->w25qx.info.MOSIPort = pDev->info.MOSIPort;
	pPrivData->w25qx.info.MOSIPin = pDev->info.MOSIPin;
	pPrivData->w25qx.info.MISOPort = pDev->info.MISOPort;
	pPrivData->w25qx.info.MISOPin = pDev->info.MISOPin;
	pPrivData->w25qx.info.CSPort = pDev->info.CSPort;
	pPrivData->w25qx.info.CSPin = pDev->info.CSPin;
	pPrivData->w25qx.info.mode = SPI_MODE_0;
	
	/* 配置软件SPI */
	spi_init(&pPrivData->w25qx);
	
	/* 函数指针赋值 */
	pDev->read_id = __w25qx_read_id;
	pDev->page_program = __w25qx_page_program;
	pDev->write_data = __w25qx_write_data;
	pDev->sector_erase = __w25qx_sector_erase;
	pDev->read_data = __w25qx_read_data;
	pDev->wakeup = __w25qx_wake_up;
	pDev->deinit = __w25qx_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	W25QX写使能
 * @param	pDev	:	W25QXDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __w25qx_write_enable(W25QXDev_t *pDev)
{
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	pPrivData->w25qx.start(&pPrivData->w25qx);								// SPI起始
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_WRITE_ENABLE);		// 交换发送写使能的指令
	pPrivData->w25qx.stop(&pPrivData->w25qx);								// SPI终止
}

/******************************************************************************
 * @brief	W25QX等待忙
 * @param	pDev	:	W25QXDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __w25qx_wait_busy(W25QXDev_t *pDev)
{
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	uint32_t Timeout;
	pPrivData->w25qx.start(&pPrivData->w25qx);							// SPI起始
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_READ_STATUS_REGISTER_1);				// 交换发送读状态寄存器1的指令
	Timeout = 100000;							// 给定超时计数时间
	while ((pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_DUMMY_BYTE) & 0x01) == 0x01)	// 循环等待忙标志位
	{
		Timeout --;								// 等待时，计数值自减
		if (Timeout == 0)						// 自减到0后，等待超时
		{
			/* 超时 */
			break;								// 跳出等待
		}
	}
	pPrivData->w25qx.stop(&pPrivData->w25qx);							// SPI终止
}

/******************************************************************************
 * @brief	W25QX读取ID号
 * @param	pDev	:	W25QXDev_t结构体指针
 * @param	mid		:	工厂ID，使用输出参数的形式返回
 * @param	did		:	设备ID，使用输出参数的形式返回
 * @return	无
 ******************************************************************************/
static void __w25qx_read_id(W25QXDev_t *pDev, uint8_t *mid, uint16_t *did)
{
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	pPrivData->w25qx.start(&pPrivData->w25qx);									// SPI起始
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_JEDEC_ID);				// 交换发送读取ID的指令
	*mid = pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_DUMMY_BYTE);		// 交换接收MID，通过输出参数返回
	*did = pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_DUMMY_BYTE);		// 交换接收DID高8位
	*did <<= 8;																	// 高8位移到高位
	*did |= pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_DUMMY_BYTE);	// 或上交换接收DID的低8位，通过输出参数返回
	pPrivData->w25qx.stop(&pPrivData->w25qx);									// SPI终止
}

/******************************************************************************
 * @brief	W25QX页编程，写入的地址范围不能跨页
 * @param	pDev		:	W25QXDev_t结构体指针
 * @param	address		:	页编程的起始地址，范围：0x000000~0x7FFFFF
 * @param	dataArray	:	用于写入数据的数组
 * @param	count		:	要写入数据的数量，范围：0~256
 * @return	无
 ******************************************************************************/
static void __w25qx_page_program(W25QXDev_t *pDev, uint32_t address, uint8_t *dataArray, uint16_t count)
{
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	uint16_t i;
	
	__w25qx_write_enable(pDev);							// 写使能
	
	pPrivData->w25qx.start(&pPrivData->w25qx);								// SPI起始
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_PAGE_PROGRAM);		// 交换发送页编程的指令
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address >> 16);			// 交换发送地址23~16位
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address >> 8);			// 交换发送地址15~8位
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address);					// 交换发送地址7~0位
	for (i = 0; i < count; i ++)						// 循环count次
	{
		pPrivData->w25qx.swap_byte(&pPrivData->w25qx, dataArray[i]);		// 依次在起始地址后写入数据
	}
	pPrivData->w25qx.stop(&pPrivData->w25qx);								// SPI终止
	
	__w25qx_wait_busy(pDev);							// 等待忙
}

/******************************************************************************
 * @brief	W25QX写入不定量数据
 * @param	pDev		:	W25QXDev_t结构体指针
 * @param	address		:	写入数据的起始地址，范围：0x000000~0x7FFFFF
 * @param	dataArray	:	写入数据的数组，通过输出参数返回
 * @param	count		:	要写入数据的数量，范围：0~0x800000
 * @return	无
 ******************************************************************************/
static void __w25qx_write_data(W25QXDev_t *pDev, uint32_t address, uint8_t *dataArray, uint32_t count)
{
	uint8_t numOfPage = 0, numOfSingle = 0, addrRem = 0, diff = 0, temp = 0;
	
	addrRem = address % W25QX_PAGE_SIZE;	// mod运算求余，若address是W25QX_PAGE_SIZE整数倍，运算结果addr值为0
	diff = W25QX_PAGE_SIZE - addrRem;		// 差diff个数据值，刚好可以对齐到页地址
	
	numOfPage =  count / W25QX_PAGE_SIZE;	// 计算出要写多少整数页
	numOfSingle = count % W25QX_PAGE_SIZE;	// mod运算求余，计算出剩余不满一页的字节数
	
	/* address刚好按页对齐 */
	if (addrRem == 0)
	{
		if (numOfPage == 0)		// 要写入的数据量小于 W25QX_PAGE_SIZE
		{
			pDev->page_program(pDev, address, dataArray, count);	// 直接写入
		}
		else					// 要写入的数据量大于 W25QX_PAGE_SIZE
		{ 
			while (numOfPage--)	// 先写入所有整数页
			{
				pDev->page_program(pDev, address, dataArray, W25QX_PAGE_SIZE);
				address +=  W25QX_PAGE_SIZE;
				dataArray += W25QX_PAGE_SIZE;
			}
			pDev->page_program(pDev, address, dataArray, numOfSingle);	// 若有多余的不满一页的数据，把它写完
		}
	}
	/* address与W25QX_PAGE_SIZE不对齐 */
	else
	{
		if (numOfPage == 0)			// 要写入的数据量小于 W25QX_PAGE_SIZE
		{
			if (numOfSingle > diff)	// 当前页剩余的diff个位置比numOfSingle小，一页写不完
			{
				temp = numOfSingle - diff;
				pDev->page_program(pDev, address, dataArray, diff);	// 先写满当前页
				address +=  diff;
				dataArray += diff;
				pDev->page_program(pDev, address, dataArray, temp);	// 再写剩余的数据
			}
			else 					// 当前页剩余的diff个位置能写完numOfSingle个数据
			{
				pDev->page_program(pDev, address, dataArray, count);
			}
		}
		else						// 要写入的数据量大于 W25QX_PAGE_SIZE
		{
			/* 地址不对齐多出的diff分开处理，不加入这个运算 */
			count -= diff;
			numOfPage =  count / W25QX_PAGE_SIZE;
			numOfSingle = count % W25QX_PAGE_SIZE;
			
			pDev->page_program(pDev, address, dataArray, diff);	// 先写完diff个数据，为的是让下一次要写的地址对齐
				
			/* 重复地址对齐 */
			address +=  diff;
			dataArray += diff;
			
			while (numOfPage--)		// 先写入所有整数页
			{
				pDev->page_program(pDev, address, dataArray, W25QX_PAGE_SIZE);
				address +=  W25QX_PAGE_SIZE;
				dataArray += W25QX_PAGE_SIZE;
			}
			if (numOfSingle != 0)
			{
				pDev->page_program(pDev, address, dataArray, numOfSingle);	// 若有多余的不满一页的数据，把它写完
			}
		}
	}
}

/******************************************************************************
 * @brief	W25QX扇区擦除（4KB）
 * @param	pDev		:	W25QXDev_t结构体指针
 * @param	address		:	指定扇区的地址，范围：0x000000~0x7FFFFF
 * @return	无
 ******************************************************************************/
static void __w25qx_sector_erase(W25QXDev_t *pDev, uint32_t address)
{
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	__w25qx_write_enable(pDev);							// 写使能
	
	pPrivData->w25qx.start(&pPrivData->w25qx);								// SPI起始
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_SECTOR_ERASE_4KB);	// 交换发送扇区擦除的指令
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address >> 16);			// 交换发送地址23~16位
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address >> 8);			// 交换发送地址15~8位
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address);					// 交换发送地址7~0位
	pPrivData->w25qx.stop(&pPrivData->w25qx);								// SPI终止
	
	__w25qx_wait_busy(pDev);							// 等待忙
}

/******************************************************************************
 * @brief	W25QX读取数据
 * @param	pDev		:	W25QXDev_t结构体指针
 * @param	address		:	读取数据的起始地址，范围：0x000000~0x7FFFFF
 * @param	dataArray	:	用于接收读取数据的数组，通过输出参数返回
 * @param	count		:	要读取数据的数量，范围：0~0x800000
 * @return	无
 ******************************************************************************/
static void __w25qx_read_data(W25QXDev_t *pDev, uint32_t address, uint8_t *dataArray, uint32_t count)
{
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	uint32_t i;
	pPrivData->w25qx.start(&pPrivData->w25qx);								// SPI起始
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_READ_DATA);			// 交换发送读取数据的指令
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address >> 16);			// 交换发送地址23~16位
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address >> 8);			// 交换发送地址15~8位
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, address);					// 交换发送地址7~0位
	for (i = 0; i < count; i ++)				// 循环count次
	{
		dataArray[i] = pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_DUMMY_BYTE);	// 依次在起始地址后读取数据
	}
	pPrivData->w25qx.stop(&pPrivData->w25qx);								// SPI终止
}

/******************************************************************************
 * @brief	唤醒W25QX
 * @param	pDev		:	W25QXDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __w25qx_wake_up(W25QXDev_t *pDev)   
{
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	pPrivData->w25qx.start(&pPrivData->w25qx);												// SPI起始
	
	pPrivData->w25qx.swap_byte(&pPrivData->w25qx, W25QX_RELEASE_POWER_DOWN_HPM_DEVICE_ID);	// 发送上电命令
	
	pPrivData->w25qx.stop(&pPrivData->w25qx);												// SPI终止
}  

/******************************************************************************
 * @brief	去初始化W25QX
 * @param	pDev   :  W25QXDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __w25qx_deinit(W25QXDev_t *pDev)
{  
    if (!pDev || !pDev->initFlag)
		return -1;
	
	W25QXPrivData_t *pPrivData = (W25QXPrivData_t *)pDev->pPrivData;
	
	/* 去初始化软件SPI */
	pPrivData->w25qx.deinit(&pPrivData->w25qx);
	
	/* 释放私有数据内存 */
	free(pDev->pPrivData);
	pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	// 修改初始化标志
	return 0;
}
