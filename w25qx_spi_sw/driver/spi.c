#include "spi.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__spi_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{spi_log("spi gpio clock no enable\r\n");} \
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
			
#elif defined(STM32F40_41xxx)

#define	__spi_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{spi_log("spi gpio clock no enable\r\n");} \
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
static void __spi_sck_write(SPIDev_t *pDev, uint8_t bitValue);
static void __spi_mosi_write(SPIDev_t *pDev, uint8_t bitValue);
static uint8_t __spi_miso_read(SPIDev_t *pDev);
static void __spi_cs_write(SPIDev_t *pDev, uint8_t bitValue);

/* 协议层 */
static void __spi_start(SPIDev_t *pDev);
static void __spi_stop(SPIDev_t *pDev);
static uint8_t __spi_swap_byte(SPIDev_t *pDev, uint8_t sendByte);
static int __spi_deinit(SPIDev_t *pDev);
										
/******************************************************************************
 * @brief	初始化软件SPI
 * @param	pDev	:	SPIDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int spi_init(SPIDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 配置时钟与GPIO */
	__spi_config_gpio_clock_enable(pDev->info.SCKPort);
	__spi_config_gpio_clock_enable(pDev->info.MOSIPort);
	__spi_config_gpio_clock_enable(pDev->info.MISOPort);
	__spi_config_gpio_clock_enable(pDev->info.CSPort);
	
	__spi_config_io_out_pp(pDev->info.SCKPort, pDev->info.SCKPin);
	__spi_config_io_out_pp(pDev->info.MOSIPort, pDev->info.MOSIPin);
	__spi_config_io_in_pu(pDev->info.MISOPort, pDev->info.MISOPin);
	__spi_config_io_out_pp(pDev->info.CSPort, pDev->info.CSPin);
	
	/* 函数指针赋值 */
	pDev->sck_write = __spi_sck_write;
	pDev->mosi_write = __spi_mosi_write;
	pDev->miso_read = __spi_miso_read;
	pDev->cs_write = __spi_cs_write;
	pDev->start = __spi_start;
	pDev->stop = __spi_stop;
	pDev->swap_byte = __spi_swap_byte;
	pDev->deinit = __spi_deinit;
	
	/* 设置默认电平 */
	__spi_cs_write(pDev, 1);				// CS默认高电平
	__spi_sck_write(pDev, 0);				// SCK默认低电平
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	软件SPI写SCK引脚电平
 * @param	pDev		:	SPIDev_t结构体指针
 * @param	bitValue	:	协议层传入的当前需要写入SCK的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_sck_write(SPIDev_t *pDev, uint8_t bitValue)
{
	__spi_io_write(pDev->info.SCKPort, pDev->info.SCKPin, bitValue);
}

/******************************************************************************
 * @brief	软件SPI写MOSI引脚电平
 * @param	pDev		:	SPIDev_t结构体指针
 * @param	bitValue	:	协议层传入的当前需要写入MOSI的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_mosi_write(SPIDev_t *pDev, uint8_t bitValue)
{
	__spi_io_write(pDev->info.MOSIPort, pDev->info.MOSIPin, bitValue);
}

/******************************************************************************
 * @brief	软件SPI读MISO引脚电平
 * @param	pDev		:	SPIDev_t结构体指针
 * @return	协议层需要得到的当前MISO的电平，范围0~1
 ******************************************************************************/
static uint8_t __spi_miso_read(SPIDev_t *pDev)
{
	return __spi_io_read(pDev->info.MISOPort, pDev->info.MISOPin);
}

/******************************************************************************
 * @brief	软件SPI写CS引脚电平
 * @param	pDev		:	SPIDev_t结构体指针
 * @param	bitValue	:	协议层传入的当前需要写入CS的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_cs_write(SPIDev_t *pDev, uint8_t bitValue)
{
	__spi_io_write(pDev->info.CSPort, pDev->info.CSPin, bitValue);
}

/******************************************************************************
 * @brief	软件SPI起始
 * @param	pDev	:  SPIDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_start(SPIDev_t *pDev)
{
	__spi_cs_write(pDev, 0);
}

/******************************************************************************
 * @brief	软件SPI停止
 * @param	pDev	:  SPIDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_stop(SPIDev_t *pDev)
{
	__spi_cs_write(pDev, 1);
}

/******************************************************************************
 * @brief	软件SPI交换一个字节
 * @param	pDev		:  SPIDev_t结构体指针
 * @param	sendByte	:  发送的字节
 * @return	接收的字节
 ******************************************************************************/
static uint8_t __spi_swap_byte(SPIDev_t *pDev, uint8_t sendByte)
{
	uint8_t recvByte = 0x00;
	
	if(pDev->info.mode == SPI_MODE_0)		// 模式0
	{
		for(uint8_t i = 0;i < 8;i++)
		{
			__spi_mosi_write(pDev, (sendByte & (0x80 >> i)));
			__spi_sck_write(pDev, 1);
			recvByte |= (__spi_miso_read(pDev) << (7 - i));
			__spi_sck_write(pDev, 0);
		}
	}
	else if(pDev->info.mode == SPI_MODE_1)	// 模式1
	{
		for(uint8_t i = 0;i < 8;i++)
		{
			__spi_sck_write(pDev, 1);
			__spi_mosi_write(pDev, (sendByte & (0x80 >> i)));
			__spi_sck_write(pDev, 0);
			recvByte |= (__spi_miso_read(pDev) << (7 - i));
		}
	}
	if(pDev->info.mode == SPI_MODE_2)		// 模式2
	{
		for(uint8_t i = 0;i < 8;i++)
		{
			__spi_mosi_write(pDev, (sendByte & (0x80 >> i)));
			__spi_sck_write(pDev, 0);
			recvByte |= (__spi_miso_read(pDev) << (7 - i));
			__spi_sck_write(pDev, 1);
		}
	}
	else if(pDev->info.mode == SPI_MODE_3)	// 模式3
	{
		for(uint8_t i = 0;i < 8;i++)
		{
			__spi_sck_write(pDev, 0);
			__spi_mosi_write(pDev, (sendByte & (0x80 >> i)));
			__spi_sck_write(pDev, 1);
			recvByte |= (__spi_miso_read(pDev) << (7 - i));
		}
	}
	
	return recvByte;
}

/******************************************************************************
 * @brief      去初始化软件SPI
 * @param[in]  pDev   :  SPIDev_t结构体指针
 * @return     0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __spi_deinit(SPIDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	pDev->initFlag = false;	// 修改初始化标志
	return 0;
}
