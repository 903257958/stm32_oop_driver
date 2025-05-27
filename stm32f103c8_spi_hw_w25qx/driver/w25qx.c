#include "w25qx.h"

/* W25QXX私有数据结构体 */
typedef struct {
	SPIDev_t spi;		// 硬件SPI设备
}W25QXPrivData_t;

/* 函数声明 */
static void __w25qx_read_id(W25QXDev_t *dev, uint8_t *mid, uint16_t *did);
static void __w25qx_page_write(W25QXDev_t *dev, uint32_t addr, uint8_t *data_array, uint16_t cnt);
static void __w25qx_write_data(W25QXDev_t *dev, uint32_t addr, uint8_t *data_array, uint32_t cnt);
static void __w25qx_sector_erase_4kb(W25QXDev_t *dev, uint32_t addr);
static void __w25qx_block_erase_64kb(W25QXDev_t *dev, uint16_t index);
static void __w25qx_read_data(W25QXDev_t *dev, uint32_t addr, uint8_t *data_array, uint32_t cnt);
static void __w25qx_wake_up(W25QXDev_t *dev);
static int __w25qx_deinit(W25QXDev_t *dev);

/******************************************************************************
 * @brief	初始化W25QX
 * @param	dev	:	W25QXDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int w25qx_init(W25QXDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 初始化私有数据 */
	dev->priv_data = (W25QXPrivData_t *)malloc(sizeof(W25QXPrivData_t));
	if (!dev->priv_data)
		return -1;
	
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	priv_data->spi.config.spix = dev->config.spix;
	priv_data->spi.config.sck_port = dev->config.sck_port;
	priv_data->spi.config.sck_pin = dev->config.sck_pin;
	priv_data->spi.config.miso_port = dev->config.miso_port;
	priv_data->spi.config.miso_pin = dev->config.miso_pin;
	priv_data->spi.config.mosi_port = dev->config.mosi_port;
	priv_data->spi.config.mosi_pin = dev->config.mosi_pin;
	priv_data->spi.config.cs_port = dev->config.cs_port;
	priv_data->spi.config.cs_pin = dev->config.cs_pin;
	priv_data->spi.config.prescaler = 2;
	priv_data->spi.config.mode = SPI_MODE_0;
	
	/* 配置硬件SPI */
	spi_hw_init(&priv_data->spi);
	
	/* 函数指针赋值 */
	dev->read_id = __w25qx_read_id;
	dev->page_write = __w25qx_page_write;
	dev->write_data = __w25qx_write_data;
	dev->sector_erase_4kb = __w25qx_sector_erase_4kb;
	dev->block_erase_64kb = __w25qx_block_erase_64kb;
	dev->read_data = __w25qx_read_data;
	dev->wakeup = __w25qx_wake_up;
	dev->deinit = __w25qx_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	W25QX写使能
 * @param	dev	:	W25QXDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __w25qx_write_enable(W25QXDev_t *dev)
{
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	priv_data->spi.start(&priv_data->spi);								// SPI起始
	priv_data->spi.swap_byte(&priv_data->spi, W25QX_WRITE_ENABLE);		// 交换发送写使能的指令
	priv_data->spi.stop(&priv_data->spi);								// SPI终止
}

/******************************************************************************
 * @brief	W25QX等待忙
 * @param	dev	:	W25QXDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __w25qx_wait_busy(W25QXDev_t *dev)
{
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	uint32_t timeout;
	priv_data->spi.start(&priv_data->spi);							// SPI起始
	priv_data->spi.swap_byte(&priv_data->spi, W25QX_READ_STATUS_REGISTER_1);				// 交换发送读状态寄存器1的指令
	timeout = 100000;							// 给定超时计数时间
	while ((priv_data->spi.swap_byte(&priv_data->spi, W25QX_DUMMY_BYTE) & 0x01) == 0x01)	// 循环等待忙标志位
	{
		timeout --;								// 等待时，计数值自减
		if (timeout == 0)						// 自减到0后，等待超时
		{
			/* 超时 */
			break;								// 跳出等待
		}
	}
	priv_data->spi.stop(&priv_data->spi);							// SPI终止
}

/******************************************************************************
 * @brief	W25QX读取ID号
 * @param	dev	:	W25QXDev_t结构体指针
 * @param	mid		:	工厂ID，使用输出参数的形式返回
 * @param	did		:	设备ID，使用输出参数的形式返回
 * @return	无
 ******************************************************************************/
static void __w25qx_read_id(W25QXDev_t *dev, uint8_t *mid, uint16_t *did)
{
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	priv_data->spi.start(&priv_data->spi);									// SPI起始
	priv_data->spi.swap_byte(&priv_data->spi, W25QX_JEDEC_ID);				// 交换发送读取ID的指令
	*mid = priv_data->spi.swap_byte(&priv_data->spi, W25QX_DUMMY_BYTE);		// 交换接收MID，通过输出参数返回
	*did = priv_data->spi.swap_byte(&priv_data->spi, W25QX_DUMMY_BYTE);		// 交换接收DID高8位
	*did <<= 8;																// 高8位移到高位
	*did |= priv_data->spi.swap_byte(&priv_data->spi, W25QX_DUMMY_BYTE);	// 或上交换接收DID的低8位，通过输出参数返回
	priv_data->spi.stop(&priv_data->spi);									// SPI终止
}

/******************************************************************************
 * @brief	W25QX页编程，写入的地址范围不能跨页，每页256字节
 * @param	dev			:	W25QXDev_t结构体指针
 * @param	addr		:	页编程的起始地址，范围：0x000000~0x7FFFFF
 * @param	data_array	:	用于写入数据的数组
 * @param	cnt			:	要写入数据的数量，范围：0~256
 * @return	无
 ******************************************************************************/
static void __w25qx_page_write(W25QXDev_t *dev, uint32_t addr, uint8_t *data_array, uint16_t cnt)
{
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	uint16_t i;
	
	__w25qx_write_enable(dev);											// 写使能
	
	priv_data->spi.start(&priv_data->spi);								// SPI起始
	priv_data->spi.swap_byte(&priv_data->spi, W25QX_PAGE_PROGRAM);		// 交换发送页编程的指令
	priv_data->spi.swap_byte(&priv_data->spi, addr >> 16);				// 交换发送地址23~16位
	priv_data->spi.swap_byte(&priv_data->spi, addr >> 8);				// 交换发送地址15~8位
	priv_data->spi.swap_byte(&priv_data->spi, addr);					// 交换发送地址7~0位
	for (i = 0; i < cnt; i ++)											// 循环cnt次
	{
		priv_data->spi.swap_byte(&priv_data->spi, data_array[i]);		// 依次在起始地址后写入数据
	}
	priv_data->spi.stop(&priv_data->spi);								// SPI终止
	
	__w25qx_wait_busy(dev);												// 等待忙
}

/******************************************************************************
 * @brief	W25QX写入不定量数据
 * @param	dev			:	W25QXDev_t结构体指针
 * @param	addr		:	写入数据的起始地址，范围：0x000000~0x7FFFFF
 * @param	data_array	:	写入数据的数组，通过输出参数返回
 * @param	cnt			:	要写入数据的数量，范围：0~0x800000
 * @return	无
 ******************************************************************************/
static void __w25qx_write_data(W25QXDev_t *dev, uint32_t addr, uint8_t *data_array, uint32_t cnt)
{
	uint8_t num_of_page = 0, num_of_single = 0, addr_rem = 0, diff = 0, temp = 0;
	
	addr_rem = addr % W25QX_PAGE_SIZE;		// mod运算求余，若addr是W25QX_PAGE_SIZE整数倍，运算结果addr值为0
	diff = W25QX_PAGE_SIZE - addr_rem;		// 差diff个数据值，刚好可以对齐到页地址
	
	num_of_page =  cnt / W25QX_PAGE_SIZE;	// 计算出要写多少整数页
	num_of_single = cnt % W25QX_PAGE_SIZE;	// mod运算求余，计算出剩余不满一页的字节数
	
	/* addr刚好按页对齐 */
	if (addr_rem == 0)
	{
		if (num_of_page == 0)		// 要写入的数据量小于 W25QX_PAGE_SIZE
		{
			__w25qx_page_write(dev, addr, data_array, cnt);	// 直接写入
		}
		else					// 要写入的数据量大于 W25QX_PAGE_SIZE
		{ 
			while (num_of_page--)	// 先写入所有整数页
			{
				__w25qx_page_write(dev, addr, data_array, W25QX_PAGE_SIZE);
				addr +=  W25QX_PAGE_SIZE;
				data_array += W25QX_PAGE_SIZE;
			}
			__w25qx_page_write(dev, addr, data_array, num_of_single);	// 若有多余的不满一页的数据，把它写完
		}
	}
	/* addr与W25QX_PAGE_SIZE不对齐 */
	else
	{
		if (num_of_page == 0)			// 要写入的数据量小于 W25QX_PAGE_SIZE
		{
			if (num_of_single > diff)	// 当前页剩余的diff个位置比num_of_single小，一页写不完
			{
				temp = num_of_single - diff;
				__w25qx_page_write(dev, addr, data_array, diff);	// 先写满当前页
				addr +=  diff;
				data_array += diff;
				__w25qx_page_write(dev, addr, data_array, temp);	// 再写剩余的数据
			}
			else 					// 当前页剩余的diff个位置能写完num_of_single个数据
			{
				__w25qx_page_write(dev, addr, data_array, cnt);
			}
		}
		else						// 要写入的数据量大于 W25QX_PAGE_SIZE
		{
			/* 地址不对齐多出的diff分开处理，不加入这个运算 */
			cnt -= diff;
			num_of_page =  cnt / W25QX_PAGE_SIZE;
			num_of_single = cnt % W25QX_PAGE_SIZE;
			
			__w25qx_page_write(dev, addr, data_array, diff);	// 先写完diff个数据，为的是让下一次要写的地址对齐
				
			/* 重复地址对齐 */
			addr +=  diff;
			data_array += diff;
			
			while (num_of_page--)		// 先写入所有整数页
			{
				__w25qx_page_write(dev, addr, data_array, W25QX_PAGE_SIZE);
				addr +=  W25QX_PAGE_SIZE;
				data_array += W25QX_PAGE_SIZE;
			}
			if (num_of_single != 0)
			{
				__w25qx_page_write(dev, addr, data_array, num_of_single);	// 若有多余的不满一页的数据，把它写完
			}
		}
	}
}

/******************************************************************************
 * @brief	W25QX扇区擦除（4KB）
 * @param	dev		:	W25QXDev_t 结构体指针
 * @param	addr	:	指定扇区的地址，范围：0x000000~0x7FFFFF
 * @return	无
 ******************************************************************************/
static void __w25qx_sector_erase_4kb(W25QXDev_t *dev, uint32_t addr)
{
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	__w25qx_write_enable(dev);						// 写使能
	
	priv_data->spi.start(&priv_data->spi);								// SPI起始
	priv_data->spi.swap_byte(&priv_data->spi, W25QX_SECTOR_ERASE_4KB);	// 交换发送扇区擦除的指令
	priv_data->spi.swap_byte(&priv_data->spi, addr >> 16);				// 交换发送地址23~16位
	priv_data->spi.swap_byte(&priv_data->spi, addr >> 8);				// 交换发送地址15~8位
	priv_data->spi.swap_byte(&priv_data->spi, addr);					// 交换发送地址7~0位
	priv_data->spi.stop(&priv_data->spi);								// SPI终止
	
	__w25qx_wait_busy(dev);							// 等待忙
}

/******************************************************************************
 * @brief	W25QX块擦除（64KB）,以W25Q64为例，8MB=8192KB，共128个block
 * @param	dev		:	W25QXDev_t 结构体指针
 * @param	index	:	指定擦除第index个块
 * @param	addr	:	指定块的地址
 * @return	无
 ******************************************************************************/
static void __w25qx_block_erase_64kb(W25QXDev_t *dev, uint16_t index)
{
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;

	__w25qx_write_enable(dev);						// 写使能
	
	priv_data->spi.start(&priv_data->spi);								// SPI起始
	priv_data->spi.swap_byte(&priv_data->spi, W25QX_SECTOR_ERASE_4KB);	// 交换发送块擦除的指令
	priv_data->spi.swap_byte(&priv_data->spi, index * 64 * 1024 >> 16);	// 交换发送地址23~16位
	priv_data->spi.swap_byte(&priv_data->spi, index * 64 * 1024 >> 8);	// 交换发送地址15~8位
	priv_data->spi.swap_byte(&priv_data->spi, index * 64 * 1024);		// 交换发送地址7~0位
	priv_data->spi.stop(&priv_data->spi);								// SPI终止
	
	__w25qx_wait_busy(dev);							// 等待忙
}

/******************************************************************************
 * @brief	W25QX读取数据
 * @param	dev			:	W25QXDev_t 结构体指针
 * @param	addr		:	读取数据的起始地址，范围：0x000000~0x7FFFFF
 * @param	data_array	:	用于接收读取数据的数组，通过输出参数返回
 * @param	cnt			:	要读取数据的数量，范围：0~0x800000
 * @return	无
 ******************************************************************************/
static void __w25qx_read_data(W25QXDev_t *dev, uint32_t addr, uint8_t *data_array, uint32_t cnt)
{
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	uint32_t i;
	priv_data->spi.start(&priv_data->spi);							// SPI起始
	priv_data->spi.swap_byte(&priv_data->spi, W25QX_READ_DATA);		// 交换发送读取数据的指令
	priv_data->spi.swap_byte(&priv_data->spi, addr >> 16);			// 交换发送地址23~16位
	priv_data->spi.swap_byte(&priv_data->spi, addr >> 8);			// 交换发送地址15~8位
	priv_data->spi.swap_byte(&priv_data->spi, addr);				// 交换发送地址7~0位
	for (i = 0; i < cnt; i++)										// 循环cnt次
	{
		data_array[i] = priv_data->spi.swap_byte(&priv_data->spi, W25QX_DUMMY_BYTE);	// 依次在起始地址后读取数据
	}
	priv_data->spi.stop(&priv_data->spi);							// SPI终止
}

/******************************************************************************
 * @brief	唤醒W25QX
 * @param	dev		:	W25QXDev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __w25qx_wake_up(W25QXDev_t *dev)   
{
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	priv_data->spi.start(&priv_data->spi);												// SPI起始
	
	priv_data->spi.swap_byte(&priv_data->spi, W25QX_RELEASE_POWER_DOWN_HPM_DEVICE_ID);	// 发送上电命令
	
	priv_data->spi.stop(&priv_data->spi);												// SPI终止
}  

/******************************************************************************
 * @brief	去初始化W25QX
 * @param	dev   :  W25QXDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __w25qx_deinit(W25QXDev_t *dev)
{  
    if (!dev || !dev->init_flag)
		return -1;
	
	W25QXPrivData_t *priv_data = (W25QXPrivData_t *)dev->priv_data;
	
	/* 去初始化硬件SPI */
	priv_data->spi.deinit(&priv_data->spi);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
	return 0;
}
