#include "spi.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__spi_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												}
													
#define	__spi_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__spi_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __spi_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)
											
#define __spi_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)
			
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__spi_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}
													
#define	__spi_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__spi_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __spi_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)
											
#define __spi_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#endif
	
/* 引脚配置层 */
static void __spi_sck_write(SPIDev_t *dev, uint8_t bit_val);
static void __spi_mosi_write(SPIDev_t *dev, uint8_t bit_val);
static uint8_t __spi_miso_read(SPIDev_t *dev);
static void __spi_cs_write(SPIDev_t *dev, uint8_t bit_val);

/* 协议层 */
static void __spi_start(SPIDev_t *dev);
static void __spi_stop(SPIDev_t *dev);
static uint8_t __spi_swap_byte(SPIDev_t *dev, uint8_t send_byte);
static int __spi_deinit(SPIDev_t *dev);
										
/******************************************************************************
 * @brief	初始化软件SPI
 * @param	dev	:	SPIDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int spi_init(SPIDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 配置时钟与GPIO */
	__spi_config_gpio_clock_enable(dev->config.sck_port);
	__spi_config_gpio_clock_enable(dev->config.miso_port);
	__spi_config_gpio_clock_enable(dev->config.mosi_port);
	__spi_config_gpio_clock_enable(dev->config.cs_port);
	
	__spi_config_io_out_pp(dev->config.sck_port, dev->config.sck_pin);
	__spi_config_io_in_pu(dev->config.miso_port, dev->config.miso_pin);
	__spi_config_io_out_pp(dev->config.mosi_port, dev->config.mosi_pin);
	__spi_config_io_out_pp(dev->config.cs_port, dev->config.cs_pin);
	
	/* 函数指针赋值 */
	dev->sck_write = __spi_sck_write;
	dev->mosi_write = __spi_mosi_write;
	dev->miso_read = __spi_miso_read;
	dev->cs_write = __spi_cs_write;
	dev->start = __spi_start;
	dev->stop = __spi_stop;
	dev->swap_byte = __spi_swap_byte;
	dev->deinit = __spi_deinit;
	
	/* 设置默认电平 */
	__spi_cs_write(dev, 1);				// CS默认高电平
	__spi_sck_write(dev, 0);			// SCK默认低电平
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	软件SPI写SCK引脚电平
 * @param	dev	:	SPIDev_t 结构体指针
 * @param	bit_val	:	协议层传入的当前需要写入SCK的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_sck_write(SPIDev_t *dev, uint8_t bit_val)
{
	__spi_io_write(dev->config.sck_port, dev->config.sck_pin, bit_val);
}

/******************************************************************************
 * @brief	软件SPI写MOSI引脚电平
 * @param	dev		:	SPIDev_t  结构体指针
 * @param	bit_val	:	协议层传入的当前需要写入MOSI的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_mosi_write(SPIDev_t *dev, uint8_t bit_val)
{
	__spi_io_write(dev->config.mosi_port, dev->config.mosi_pin, bit_val);
}

/******************************************************************************
 * @brief	软件SPI读MISO引脚电平
 * @param	dev		:	SPIDev_t 结构体指针
 * @return	协议层需要得到的当前MISO的电平，范围0~1
 ******************************************************************************/
static uint8_t __spi_miso_read(SPIDev_t *dev)
{
	return __spi_io_read(dev->config.miso_port, dev->config.miso_pin);
}

/******************************************************************************
 * @brief	软件SPI写CS引脚电平
 * @param	dev		:	SPIDev_t 结构体指针
 * @param	bit_val	:	协议层传入的当前需要写入CS的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_cs_write(SPIDev_t *dev, uint8_t bit_val)
{
	__spi_io_write(dev->config.cs_port, dev->config.cs_pin, bit_val);
}

/******************************************************************************
 * @brief	软件SPI起始
 * @param	dev	:  SPIDev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_start(SPIDev_t *dev)
{
	__spi_cs_write(dev, 0);
}

/******************************************************************************
 * @brief	软件SPI停止
 * @param	dev	:  SPIDev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_stop(SPIDev_t *dev)
{
	__spi_cs_write(dev, 1);
}

/******************************************************************************
 * @brief	软件SPI交换一个字节
 * @param	dev			:  SPIDev_t 结构体指针
 * @param	send_byte	:  发送的字节
 * @return	接收的字节
 ******************************************************************************/
static uint8_t __spi_swap_byte(SPIDev_t *dev, uint8_t send_byte)
{
	uint8_t recvByte = 0x00;
	
	if(dev->config.mode == SPI_MODE_0)		// 模式0
	{
		for(uint8_t i = 0;i < 8;i++)
		{
			__spi_mosi_write(dev, (send_byte & (0x80 >> i)));
			__spi_sck_write(dev, 1);
			recvByte |= (__spi_miso_read(dev) << (7 - i));
			__spi_sck_write(dev, 0);
		}
	}
	else if(dev->config.mode == SPI_MODE_1)	// 模式1
	{
		for(uint8_t i = 0;i < 8;i++)
		{
			__spi_sck_write(dev, 1);
			__spi_mosi_write(dev, (send_byte & (0x80 >> i)));
			__spi_sck_write(dev, 0);
			recvByte |= (__spi_miso_read(dev) << (7 - i));
		}
	}
	if(dev->config.mode == SPI_MODE_2)		// 模式2
	{
		for(uint8_t i = 0;i < 8;i++)
		{
			__spi_mosi_write(dev, (send_byte & (0x80 >> i)));
			__spi_sck_write(dev, 0);
			recvByte |= (__spi_miso_read(dev) << (7 - i));
			__spi_sck_write(dev, 1);
		}
	}
	else if(dev->config.mode == SPI_MODE_3)	// 模式3
	{
		for(uint8_t i = 0;i < 8;i++)
		{
			__spi_sck_write(dev, 0);
			__spi_mosi_write(dev, (send_byte & (0x80 >> i)));
			__spi_sck_write(dev, 1);
			recvByte |= (__spi_miso_read(dev) << (7 - i));
		}
	}
	
	return recvByte;
}

/******************************************************************************
 * @brief      去初始化软件SPI
 * @param[in]  dev   :  SPIDev_t 结构体指针
 * @return     0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __spi_deinit(SPIDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	return 0;
}
