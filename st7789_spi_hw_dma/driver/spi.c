#include "spi.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define	__spi_config_clock_enable(SPIx)		{	if(SPIx == SPI1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);} \
												else if(SPIx == SPI2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);} \
												else if(SPIx == SPI3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);} \
												else					{spi_log("spi clock no enable\r\n");} \
											}

#define	__spi_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{spi_log("spi gpio clock no enable\r\n");} \
												}
								
#define	__spi_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
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

#define __spi_io_write(port ,pin, value)	GPIO_WriteBit(port ,pin, (BitAction)value)

#define __spi_get_prescaler(prescaler)	(	prescaler == 2 ? SPI_BaudRatePrescaler_2 : \
											prescaler == 4 ? SPI_BaudRatePrescaler_4 : \
											prescaler == 8 ? SPI_BaudRatePrescaler_8 : \
											prescaler == 16 ? SPI_BaudRatePrescaler_16 : \
											prescaler == 32 ? SPI_BaudRatePrescaler_32 : \
											prescaler == 64 ? SPI_BaudRatePrescaler_64 : \
											prescaler == 128 ? SPI_BaudRatePrescaler_128 : \
											prescaler == 256 ? SPI_BaudRatePrescaler_256 : \
											(int)0	)
						
#elif defined(STM32F40_41xxx)

#define	__spi_config_clock_enable(SPIx)		{	if(SPIx == SPI1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);} \
												else if(SPIx == SPI2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);} \
												else if(SPIx == SPI3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);} \
												else					{spi_log("spi clock no enable\r\n");} \
											}

#define	__spi_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{spi_log("spi gpio clock no enable\r\n");} \
												}
								
#define	__spi_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
													
#define	__spi_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __spi_get_gpio_pin_sourse(pin)	(	pin == GPIO_Pin_0 ? GPIO_PinSource0 : \
											pin == GPIO_Pin_1 ? GPIO_PinSource1 : \
											pin == GPIO_Pin_2 ? GPIO_PinSource2 : \
											pin == GPIO_Pin_3 ? GPIO_PinSource3 : \
											pin == GPIO_Pin_4 ? GPIO_PinSource4 : \
											pin == GPIO_Pin_5 ? GPIO_PinSource5 : \
											pin == GPIO_Pin_6 ? GPIO_PinSource6 : \
											pin == GPIO_Pin_7 ? GPIO_PinSource7 : \
											pin == GPIO_Pin_8 ? GPIO_PinSource8 : \
											pin == GPIO_Pin_9 ? GPIO_PinSource9 : \
											pin == GPIO_Pin_10 ? GPIO_PinSource10 : \
											pin == GPIO_Pin_11 ? GPIO_PinSource11 : \
											pin == GPIO_Pin_12 ? GPIO_PinSource12 : \
											pin == GPIO_Pin_13 ? GPIO_PinSource13 : \
											pin == GPIO_Pin_14 ? GPIO_PinSource14 : \
											pin == GPIO_Pin_15 ? GPIO_PinSource15 : \
											(int)0	)
												
#define __spi_get_gpio_af(spix)	(	spix == SPI1 ? GPIO_AF_SPI1 : \
									spix == SPI2 ? GPIO_AF_SPI2 : \
									spix == SPI3 ? GPIO_AF_SPI3 : \
									(int)0	)

#define __spi_io_write(port ,pin, value)	GPIO_WriteBit(port ,pin, (BitAction)value)

#define __spi_get_prescaler(prescaler)	(	prescaler == 2 ? SPI_BaudRatePrescaler_2 : \
											prescaler == 4 ? SPI_BaudRatePrescaler_4 : \
											prescaler == 8 ? SPI_BaudRatePrescaler_8 : \
											prescaler == 16 ? SPI_BaudRatePrescaler_16 : \
											prescaler == 32 ? SPI_BaudRatePrescaler_32 : \
											prescaler == 64 ? SPI_BaudRatePrescaler_64 : \
											prescaler == 128 ? SPI_BaudRatePrescaler_128 : \
											prescaler == 256 ? SPI_BaudRatePrescaler_256 : \
											(int)0	)

#endif					
										
/* 引脚配置层 */
static void __spi_cs_write(SPIDev_t *pDev, uint8_t bitValue);

/* 协议层 */
static void __spi_start(SPIDev_t *pDev);
static void __spi_stop(SPIDev_t *pDev);
static uint8_t __spi_swap_byte(SPIDev_t *pDev, uint8_t sendByte);
static int __spi_deinit(SPIDev_t *pDev);
										
/******************************************************************************
 * @brief	初始化硬件SPI
 * @param	pDev	:	SPIDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int spi_init(SPIDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 配置时钟与GPIO */
	__spi_config_clock_enable(pDev->info.spix);
	__spi_config_gpio_clock_enable(pDev->info.SCKPort);
	__spi_config_gpio_clock_enable(pDev->info.MOSIPort);
	__spi_config_gpio_clock_enable(pDev->info.MISOPort);
	__spi_config_gpio_clock_enable(pDev->info.CSPort);
	
	__spi_config_io_af_pp(pDev->info.SCKPort, pDev->info.SCKPin);
	__spi_config_io_af_pp(pDev->info.MOSIPort, pDev->info.MOSIPin);
	__spi_config_io_af_pp(pDev->info.MISOPort, pDev->info.MISOPin);
	__spi_config_io_out_pp(pDev->info.CSPort, pDev->info.CSPin);
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	/* STM32F1的PB3、PB4、PA15为JTAG引脚，配置SPI3时需要解除JTAG */
	if(pDev->info.spix == SPI3)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		DBGMCU->CR &= ~((uint32_t)1 << 5);
	}
	
	#elif defined(STM32F40_41xxx)
	
	/* STM32F4配置为复用输出时需要配置引脚复用映射 */
	GPIO_PinAFConfig(	pDev->info.SCKPort, 
						__spi_get_gpio_pin_sourse(pDev->info.SCKPin), 
						__spi_get_gpio_af(pDev->info.spix)	);
						
	GPIO_PinAFConfig(	pDev->info.MOSIPort, 
						__spi_get_gpio_pin_sourse(pDev->info.MOSIPin), 
						__spi_get_gpio_af(pDev->info.spix)	);
						
	GPIO_PinAFConfig(	pDev->info.MISOPort, 
						__spi_get_gpio_pin_sourse(pDev->info.MISOPin), 
						__spi_get_gpio_af(pDev->info.spix)	);
	
	#endif
		
	/* 配置硬件SPI */
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;											// 主机
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;						// 双线全双工
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;										// 8位数据帧
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;										// 高位先行
	SPI_InitStructure.SPI_BaudRatePrescaler = __spi_get_prescaler(pDev->info.prescaler);	// 分频系数
	if(pDev->info.mode == SPI_MODE_0)											// 模式0
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	}
	else if(pDev->info.mode == SPI_MODE_1)										// 模式1
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	}
	else if(pDev->info.mode == SPI_MODE_2)										// 模式2
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	}
	else if(pDev->info.mode == SPI_MODE_3)										// 模式3
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	}
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;									// 软件NSS
	SPI_InitStructure.SPI_CRCPolynomial = 7;									// CRC，不使用
	SPI_Init(pDev->info.spix, &SPI_InitStructure);
	
	/* 开启硬件SPI */
	SPI_Cmd(pDev->info.spix, ENABLE);
	
	/* 函数指针赋值 */
	pDev->cs_write = __spi_cs_write;
	pDev->start = __spi_start;
	pDev->stop = __spi_stop;
	pDev->swap_byte = __spi_swap_byte;
	pDev->deinit = __spi_deinit;
	
	/* 设置默认电平 */
	__spi_cs_write(pDev, 1);
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	硬件SPI写CS引脚电平
 * @param	pDev		:	SPIDev_t结构体指针
 * @param	bitValue	:	协议层传入的当前需要写入CS的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_cs_write(SPIDev_t *pDev, uint8_t bitValue)
{
	__spi_io_write(pDev->info.CSPort, pDev->info.CSPin, bitValue);
}

/******************************************************************************
 * @brief	硬件SPI起始
 * @param	pDev	:  SPIDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_start(SPIDev_t *pDev)
{
	__spi_cs_write(pDev, 0);
}

/******************************************************************************
 * @brief	硬件SPI停止
 * @param	pDev	:  SPIDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_stop(SPIDev_t *pDev)
{
	__spi_cs_write(pDev, 1);
}

/******************************************************************************
 * @brief	硬件SPI交换一个字节
 * @param	pDev		:  SPIDev_t结构体指针
 * @param	sendByte	:  发送的字节
 * @return	接收的字节
 ******************************************************************************/
static uint8_t __spi_swap_byte(SPIDev_t *pDev, uint8_t sendByte)
{
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD) || defined(STM32F40_41xxx)
	
	while(SPI_I2S_GetFlagStatus(pDev->info.spix, SPI_I2S_FLAG_TXE) != SET);		// 等待TXE置1，表示发送寄存器为空，发送一个字节
	SPI_I2S_SendData(pDev->info.spix, sendByte);								// 发送字节
	while(SPI_I2S_GetFlagStatus(pDev->info.spix, SPI_I2S_FLAG_RXNE) != SET);	// 等待RXNE置1，表示接收寄存器非空，收到一个字节
	
	return SPI_I2S_ReceiveData(pDev->info.spix);	// 读取接收到的字节
	
	#endif
}

/******************************************************************************
 * @brief      去初始化硬件SPI
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
