#include "drv_w25qx.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void w25qx_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_W25QX_PLATFORM_STM32F1
#define RCC_CMD(port) RCC_APB2PeriphClockCmd(RCC_APB2Periph_##port, ENABLE)
#elif DRV_W25QX_PLATFORM_STM32F4
#define RCC_CMD(port) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_##port, ENABLE)
#elif DRV_W25QX_PLATFORM_GD32F1
#define RCC_CMD(port) rcu_periph_clock_enable(RCU_##port)
#endif
	switch ((uint32_t)port) {
    case (uint32_t)GPIOA: RCC_CMD(GPIOA); break;
    case (uint32_t)GPIOB: RCC_CMD(GPIOB); break;
	case (uint32_t)GPIOC: RCC_CMD(GPIOC); break;
	case (uint32_t)GPIOD: RCC_CMD(GPIOD); break;
	case (uint32_t)GPIOE: RCC_CMD(GPIOE); break;
	case (uint32_t)GPIOF: RCC_CMD(GPIOF); break;
	case (uint32_t)GPIOG: RCC_CMD(GPIOG); break;
	}
}

/**
 * @brief	初始化 GPIO
 * @param[in] cfg w25qx_cfg_t 结构体指针
 */
static void w25qx_hw_gpio_init(const w25qx_cfg_t *cfg)
{
#if DRV_W25QX_PLATFORM_STM32F1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->cs_pin;
	GPIO_Init(cfg->cs_port, &GPIO_InitStructure);
#elif DRV_W25QX_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->cs_pin;
	GPIO_Init(cfg->cs_port, &GPIO_InitStructure);
#elif DRV_W25QX_PLATFORM_GD32F1
	gpio_init(cfg->cs_port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, cfg->cs_pin);
#endif
}

/**
 * @brief	写 GPIO 引脚电平
 * @param[in] port  端口
 * @param[in] pin   引脚
 * @param[in] level 电平
 */
static inline void w25qx_hw_gpio_write_bit(gpio_port_t port, gpio_pin_t pin, uint8_t level)
{
#if DRV_W25QX_PLATFORM_STM32F1 || DRV_W25QX_PLATFORM_STM32F4
	GPIO_WriteBit(port, pin, (BitAction)level);
#elif DRV_W25QX_PLATFORM_GD32F1
	gpio_bit_write(port, pin, (bit_status)(level));
#endif
}

/**
 * @brief   初始化 W25QX 硬件
 * @param[in] cfg w25qx_cfg_t 结构体指针
 */
static void w25qx_hw_init(const w25qx_cfg_t *cfg)
{
	w25qx_hw_gpio_clock_enable(cfg->cs_port);
	
	w25qx_hw_gpio_init(cfg);

	w25qx_hw_gpio_write_bit(cfg->cs_port, cfg->cs_pin, 1);
}

/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int w25qx_read_id_impl(w25qx_dev_t *dev, uint8_t *mid, uint16_t *did);
static int w25qx_write_page_impl(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data);
static int w25qx_write_data_impl(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data);
static int w25qx_erase_sector_4kb_impl(w25qx_dev_t *dev, uint32_t addr);
static int w25qx_erase_block_64kb_impl(w25qx_dev_t *dev, uint16_t index);
static int w25qx_read_data_impl(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data);
static int w25qx_wake_up_impl(w25qx_dev_t *dev);
static int w25qx_deinit_impl(w25qx_dev_t *dev);

/* 操作接口表 */
static const w25qx_ops_t w25qx_ops = {
	.read_id          = w25qx_read_id_impl,
	.write_page       = w25qx_write_page_impl,
	.write_data       = w25qx_write_data_impl,
	.erase_sector_4kb = w25qx_erase_sector_4kb_impl,
	.erase_block_64kb = w25qx_erase_block_64kb_impl,
	.read_data        = w25qx_read_data_impl,
	.wakeup           = w25qx_wake_up_impl,
	.deinit 		  = w25qx_deinit_impl
};

/**
 * @brief   初始化 W25QX 驱动
 * @param[out] dev w25qx_dev_t 结构体指针
 * @param[in]  cfg w25qx_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_w25qx_init(w25qx_dev_t *dev, const w25qx_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

    dev->cfg = *cfg;
	dev->ops = &w25qx_ops;

	w25qx_hw_init(cfg);
	return 0;
}

/**
 * @brief   W25QX 写使能
 * @param[in] dev w25qx_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_write_enable(w25qx_dev_t *dev)
{
	if (!dev)
        return -EINVAL;
	
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI起始
	dev->cfg.spi_ops->swap_byte(W25QX_WRITE_ENABLE, NULL);		// 交换发送写使能的指令
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI终止
	return 0;
}

/**
 * @brief   W25QX 等待忙
 * @param[in] dev w25qx_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_wait_busy(w25qx_dev_t *dev)
{
	if (!dev)
        return -EINVAL;
	
	uint32_t timeout;
	uint8_t recv;
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);			// SPI起始
	dev->cfg.spi_ops->swap_byte(W25QX_READ_STATUS_REGISTER_1, NULL);	// 交换发送读状态寄存器1的指令
	timeout = 1000000;

	while (timeout--) {
		dev->cfg.spi_ops->swap_byte(W25QX_DUMMY_BYTE, &recv);
		if ((recv & 0x01) == 0)
			break;
	}
	if (timeout == 0)
		return -ETIMEDOUT;
	
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);			// SPI终止
	return 0;
}

/**
 * @brief   W25QX 读取 ID 号
 * @param[in]  dev w25qx_dev_t 结构体指针
 * @param[out] mid 工厂 ID
 * @param[out] did 设备 ID
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_read_id_impl(w25qx_dev_t *dev, uint8_t *mid, uint16_t *did)
{
	if (!dev)
        return -EINVAL;
	
	uint8_t did_high;
	uint8_t did_low;

	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI起始
	dev->cfg.spi_ops->swap_byte(W25QX_JEDEC_ID, NULL);			// 交换发送读取ID的指令
	dev->cfg.spi_ops->swap_byte(W25QX_DUMMY_BYTE, mid);			// 交换接收MID
	dev->cfg.spi_ops->swap_byte(W25QX_DUMMY_BYTE, &did_high);	// 交换接收DID高8位
	dev->cfg.spi_ops->swap_byte(W25QX_DUMMY_BYTE, &did_low);	// 交换接收DID低8位
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI终止

	*did = (did_high << 8) | did_low;
	return 0;
}

/**
 * @brief   W25QX 按页写入，写入的地址范围不能跨页，每页 256 字节
 * @param[in] dev  w25qx_dev_t 结构体指针
 * @param[in] addr 起始地址
 * @param[in] cnt  要写入数据的数量
 * @param[in] data 用于写入数据的数组
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_write_page_impl(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data)
{
	if (!dev)
        return -EINVAL;

	w25qx_write_enable(dev);									// 写使能
	
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI起始
	dev->cfg.spi_ops->swap_byte(W25QX_PAGE_PROGRAM, NULL);		// 交换发送页编程的指令
	dev->cfg.spi_ops->swap_byte(addr >> 16, NULL);				// 交换发送地址23~16位
	dev->cfg.spi_ops->swap_byte(addr >> 8, NULL);				// 交换发送地址15~8位
	dev->cfg.spi_ops->swap_byte(addr, NULL);					// 交换发送地址7~0位
	for (uint16_t i = 0; i < cnt; i++)							// 循环cnt次
		dev->cfg.spi_ops->swap_byte(data[i], NULL);				// 依次在起始地址后写入数据
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI终止
	
	w25qx_wait_busy(dev);										// 等待忙
	return 0;
}

/**
 * @brief   W25QX 写入不定量数据
 * @param[in] dev  w25qx_dev_t 结构体指针
 * @param[in] addr 起始地址
 * @param[in] cnt  要写入数据的数量
 * @param[in] data 用于写入数据的数组
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_write_data_impl(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data)
{
	uint8_t num_of_page = 0, num_of_single = 0, addr_rem = 0, diff = 0, temp = 0;
	
	addr_rem = addr % W25QX_PAGE_SIZE;		// mod运算求余，若addr是W25QX_PAGE_SIZE整数倍，运算结果addr值为0
	diff = W25QX_PAGE_SIZE - addr_rem;		// 差diff个数据值，刚好可以对齐到页地址
	
	num_of_page = cnt / W25QX_PAGE_SIZE;	// 计算出要写多少整数页
	num_of_single = cnt % W25QX_PAGE_SIZE;	// mod运算求余，计算出剩余不满一页的字节数
	
	/* addr刚好按页对齐 */
	if (addr_rem == 0) {
		if (num_of_page == 0) {
			/* 要写入的数据量小于 W25QX_PAGE_SIZE */
			w25qx_write_page_impl(dev, addr, cnt, data);	// 直接写入
		} else { 
			/* 要写入的数据量大于 W25QX_PAGE_SIZE */
			while (num_of_page--) {							// 先写入所有整数页
				w25qx_write_page_impl(dev, addr, W25QX_PAGE_SIZE, data);
				addr +=  W25QX_PAGE_SIZE;
				data += W25QX_PAGE_SIZE;
			}
			w25qx_write_page_impl(dev, addr, num_of_single, data);	// 若有多余的不满一页的数据，把它写完
		}
	
	/* addr与W25QX_PAGE_SIZE不对齐 */
	} else {
		if (num_of_page == 0) {
			/* 要写入的数据量小于 W25QX_PAGE_SIZE */
			if (num_of_single > diff) {
				/* 当前页剩余的diff个位置比num_of_single小，一页写不完 */
				temp = num_of_single - diff;
				w25qx_write_page_impl(dev, addr, diff, data);	// 先写满当前页
				addr +=  diff;
				data += diff;
				w25qx_write_page_impl(dev, addr, temp, data);	// 再写剩余的数据
			} else {
				/* 当前页剩余的diff个位置能写完num_of_single个数据 */
				w25qx_write_page_impl(dev, addr, cnt, data);
			}
		} else {
			/* 要写入的数据量大于 W25QX_PAGE_SIZE */
			/* 地址不对齐多出的diff分开处理，不加入这个运算 */
			cnt -= diff;
			num_of_page =  cnt / W25QX_PAGE_SIZE;
			num_of_single = cnt % W25QX_PAGE_SIZE;
			
			w25qx_write_page_impl(dev, addr, diff, data);		// 先写完diff个数据，为的是让下一次要写的地址对齐
				
			/* 重复地址对齐 */
			addr +=  diff;
			data += diff;
			
			while (num_of_page--) {	// 先写入所有整数页
				w25qx_write_page_impl(dev, addr, W25QX_PAGE_SIZE, data);
				addr +=  W25QX_PAGE_SIZE;
				data += W25QX_PAGE_SIZE;
			}
			if (num_of_single != 0)
				w25qx_write_page_impl(dev, addr, num_of_single, data);	// 若有多余的不满一页的数据，把它写完
		}
	}
	return 0;
}

/**
 * @brief   W25QX 扇区擦除（4KB）
 * @param[in] dev  w25qx_dev_t 结构体指针
 * @param[in] addr 指定扇区的地址
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_erase_sector_4kb_impl(w25qx_dev_t *dev, uint32_t addr)
{
	if (!dev)
        return -EINVAL;
	
	w25qx_write_enable(dev);									// 写使能
	
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI起始
	dev->cfg.spi_ops->swap_byte(W25QX_SECTOR_ERASE_4KB, NULL);	// 交换发送扇区擦除的指令
	dev->cfg.spi_ops->swap_byte(addr >> 16, NULL);				// 交换发送地址23~16位
	dev->cfg.spi_ops->swap_byte(addr >> 8, NULL);				// 交换发送地址15~8位
	dev->cfg.spi_ops->swap_byte(addr, NULL);					// 交换发送地址7~0位
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI终止
	
	w25qx_wait_busy(dev);										// 等待忙
	return 0;
}

/**
 * @brief   W25QX 块擦除（64KB）
 * @details 以 W25Q64 为例，8MB=8192KB，共128个block
 * @param[in] dev   w25qx_dev_t 结构体指针
 * @param[in] index 指定擦除块的索引
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_erase_block_64kb_impl(w25qx_dev_t *dev, uint16_t index)
{
	if (!dev)
        return -EINVAL;

	w25qx_write_enable(dev);									// 写使能
	
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI起始
	dev->cfg.spi_ops->swap_byte(W25QX_BLOCK_ERASE_64KB, NULL);	// 交换发送块擦除的指令
	dev->cfg.spi_ops->swap_byte(index * 64 * 1024 >> 16, NULL);	// 交换发送地址23~16位
	dev->cfg.spi_ops->swap_byte(index * 64 * 1024 >> 8, NULL);	// 交换发送地址15~8位
	dev->cfg.spi_ops->swap_byte(index * 64 * 1024, NULL);		// 交换发送地址7~0位
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// SPI终止
	
	w25qx_wait_busy(dev);										// 等待忙
	return 0;
}

/**
 * @brief   W25QX 读取数据
 * @param[in]  dev  w25qx_dev_t 结构体指针
 * @param[in]  addr 读取数据的起始地址
 * @param[in]  cnt  要读取数据的数量
 * @param[out] data 用于接收读取数据的数组
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_read_data_impl(w25qx_dev_t *dev, uint32_t addr, uint32_t cnt, uint8_t *data)
{
	if (!dev)
        return -EINVAL;

	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);		// SPI起始
	dev->cfg.spi_ops->swap_byte(W25QX_READ_DATA, NULL);				// 交换发送读取数据的指令
	dev->cfg.spi_ops->swap_byte(addr >> 16, NULL);					// 交换发送地址23~16位
	dev->cfg.spi_ops->swap_byte(addr >> 8, NULL);					// 交换发送地址15~8位
	dev->cfg.spi_ops->swap_byte(addr, NULL);						// 交换发送地址7~0位
	for (uint32_t i = 0; i < cnt; i++)
		dev->cfg.spi_ops->swap_byte(W25QX_DUMMY_BYTE, &data[i]);	// 依次在起始地址后读取数据
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);		// SPI终止
	return 0;
}

/**
 * @brief   W25QX 唤醒
 * @param[in] dev w25qx_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_wake_up_impl(w25qx_dev_t *dev)   
{
	if (!dev)
        return -EINVAL;
	
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);					// SPI起始
	dev->cfg.spi_ops->swap_byte(W25QX_RELEASE_POWER_DOWN_HPM_DEVICE_ID, NULL);	// 发送上电命令
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);					// SPI终止
	return 0;
}  

/**
 * @brief   去初始化 W25QX
 * @param[in] dev w25qx_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int w25qx_deinit_impl(w25qx_dev_t *dev)
{  
    if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
