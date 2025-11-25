#include "spi_soft.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define	__spi_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
										else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
										else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
										else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
										else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
										else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
										else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
									}
													
#define	__spi_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__spi_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __spi_io_write(port ,pin, value)	GPIO_WriteBit(port ,pin, (BitAction)value)

#define __spi_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE)

#define	__spi_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
										else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
										else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
										else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
										else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
										else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
										else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
									}
													
#define	__spi_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__spi_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __spi_io_write(port ,pin, value)	GPIO_WriteBit(port ,pin, (BitAction)value)

#define __spi_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)

#define	__spi_io_clock_enable(port)	{	if (port == GPIOA)		{rcu_periph_clock_enable(RCU_GPIOA);} \
										else if (port == GPIOB)	{rcu_periph_clock_enable(RCU_GPIOB);} \
										else if (port == GPIOC)	{rcu_periph_clock_enable(RCU_GPIOC);} \
										else if (port == GPIOD)	{rcu_periph_clock_enable(RCU_GPIOD);} \
										else if (port == GPIOE)	{rcu_periph_clock_enable(RCU_GPIOE);} \
										else if (port == GPIOF)	{rcu_periph_clock_enable(RCU_GPIOF);} \
										else if (port == GPIOG)	{rcu_periph_clock_enable(RCU_GPIOG);} \
									}
													
#define	__spi_config_io_out_pp(port, pin)	gpio_init(port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin);

#define	__spi_config_io_in_pu(port, pin)	gpio_init(port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pin);

#define __spi_io_write(port, pin, value)	gpio_bit_write(port, pin, (bit_status)value);

#define __spi_io_read(port, pin)		    gpio_input_bit_get(port, pin);

#else
	#error spi_soft.c: No processor defined!
#endif

#endif
										
/* 引脚配置层 */
static void __spi_sck_write(spi_soft_dev_t *dev, uint8_t bit_val);
static void __spi_mosi_write(spi_soft_dev_t *dev, uint8_t bit_val);
static uint8_t __spi_miso_read(spi_soft_dev_t *dev);
static void __spi_cs_write(spi_soft_dev_t *dev, uint8_t bit_val);

/* 协议层 */
static void __spi_start(spi_soft_dev_t *dev);
static void __spi_stop(spi_soft_dev_t *dev);
static uint8_t __spi_swap_byte(spi_soft_dev_t *dev, uint8_t send_byte);
static int8_t __spi_deinit(spi_soft_dev_t *dev);
										
/******************************************************************************
 * @brief	初始化软件SPI
 * @param	dev	:	spi_soft_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t spi_soft_init(spi_soft_dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 开启时钟 */
	__spi_io_clock_enable(dev->config.sck_port);
	__spi_io_clock_enable(dev->config.mosi_port);
	__spi_io_clock_enable(dev->config.miso_port);
	__spi_io_clock_enable(dev->config.cs_port);
	
	/* 配置GPIO */
	__spi_config_io_out_pp(dev->config.sck_port, dev->config.sck_pin);
	__spi_config_io_in_pu(dev->config.miso_port, dev->config.miso_pin);
	__spi_config_io_out_pp(dev->config.mosi_port, dev->config.mosi_pin);
	__spi_config_io_out_pp(dev->config.cs_port, dev->config.cs_pin);
	
	/* 函数指针赋值 */
	dev->cs_write = __spi_cs_write;
	dev->start = __spi_start;
	dev->stop = __spi_stop;
	dev->swap_byte = __spi_swap_byte;
	dev->deinit = __spi_deinit;
	
	dev->init_flag = true;

	/* 设置默认电平 */
	__spi_cs_write(dev, 1);			// CS默认高电平
	if (dev->config.mode == SPI_MODE_0 || dev->config.mode == SPI_MODE_1)
	{
		__spi_sck_write(dev, 0);	// SCK默认低电平
	}
	else if (dev->config.mode == SPI_MODE_2 || dev->config.mode == SPI_MODE_3)
	{
		__spi_sck_write(dev, 1);	// SCK默认高电平
	}
	
	return 0;
}

/******************************************************************************
 * @brief	软件SPI写SCK引脚电平
 * @param	dev		:	spi_soft_dev_t 结构体指针
 * @param	bit_val	:	协议层传入的当前需要写入SCK的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_sck_write(spi_soft_dev_t *dev, uint8_t bit_val)
{
	__spi_io_write(dev->config.sck_port, dev->config.sck_pin, bit_val);
}

/******************************************************************************
 * @brief	软件SPI写MOSI引脚电平
 * @param	dev		:	spi_soft_dev_t 结构体指针
 * @param	bit_val	:	协议层传入的当前需要写入MOSI的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_mosi_write(spi_soft_dev_t *dev, uint8_t bit_val)
{
	__spi_io_write(dev->config.mosi_port, dev->config.mosi_pin, bit_val);
}

/******************************************************************************
 * @brief	软件SPI读MISO引脚电平
 * @param	dev	:	spi_soft_dev_t 结构体指针
 * @return	协议层需要得到的当前MISO的电平，范围0~1
 ******************************************************************************/
static uint8_t __spi_miso_read(spi_soft_dev_t *dev)
{
	return __spi_io_read(dev->config.miso_port, dev->config.miso_pin);
}

/******************************************************************************
 * @brief	软件SPI写CS引脚电平
 * @param	dev		:	spi_soft_dev_t 结构体指针
 * @param	bit_val	:	协议层传入的当前需要写入CS的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_cs_write(spi_soft_dev_t *dev, uint8_t bit_val)
{
	__spi_io_write(dev->config.cs_port, dev->config.cs_pin, bit_val);
}

/******************************************************************************
 * @brief	软件SPI起始
 * @param	dev	:	spi_soft_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_start(spi_soft_dev_t *dev)
{
	__spi_cs_write(dev, 0);
}

/******************************************************************************
 * @brief	软件SPI停止
 * @param	dev	:	spi_soft_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_stop(spi_soft_dev_t *dev)
{
	__spi_cs_write(dev, 1);
}

/******************************************************************************
 * @brief	软件SPI交换一个字节
 * @param	dev			:	spi_soft_dev_t 结构体指针
 * @param	send_byte	:	发送的字节
 * @return	接收的字节
 ******************************************************************************/
static uint8_t __spi_swap_byte(spi_soft_dev_t *dev, uint8_t send_byte)
{
	uint8_t i;
	uint8_t recv_byte = 0x00;
	
	if (dev->config.mode == SPI_MODE_0)
	{
		/* 模式0 */
		for (i = 0; i < 8; i++)
		{
			__spi_mosi_write(dev, (send_byte & (0x80 >> i)));
			__spi_sck_write(dev, 1);
			recv_byte |= (__spi_miso_read(dev) << (7 - i));
			__spi_sck_write(dev, 0);
		}
	}
	else if (dev->config.mode == SPI_MODE_1)
	{
		/* 模式1 */
		for (i = 0; i < 8; i++)
		{
			__spi_sck_write(dev, 1);
			__spi_mosi_write(dev, (send_byte & (0x80 >> i)));
			__spi_sck_write(dev, 0);
			recv_byte |= (__spi_miso_read(dev) << (7 - i));
		}
	}
	if (dev->config.mode == SPI_MODE_2)
	{
		/* 模式2 */
		for (i = 0; i < 8; i++)
		{
			__spi_mosi_write(dev, (send_byte & (0x80 >> i)));
			__spi_sck_write(dev, 0);
			recv_byte |= (__spi_miso_read(dev) << (7 - i));
			__spi_sck_write(dev, 1);
		}
	}
	else if (dev->config.mode == SPI_MODE_3)
	{
		/* 模式3 */
		for (i = 0; i < 8; i++)
		{
			__spi_sck_write(dev, 0);
			__spi_mosi_write(dev, (send_byte & (0x80 >> i)));
			__spi_sck_write(dev, 1);
			recv_byte |= (__spi_miso_read(dev) << (7 - i));
		}
	}
	
	return recv_byte;
}

/******************************************************************************
 * @brief	去初始化软件SPI
 * @param	dev	:	spi_soft_dev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __spi_deinit(spi_soft_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	return 0;
}
