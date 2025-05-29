#include "spi_hard.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define	__spi_clock_enable(SPIx)		{	if (SPIx == SPI1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);} \
											else if (SPIx == SPI2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);} \
											else if (SPIx == SPI3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);} \
										}

#define	__spi_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
										else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
										else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
										else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
										else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
										else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
										else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
									}
								
#define	__spi_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
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

#if defined(STM32F40_41xxx) || defined(STM32F411xE)

#define	__spi_clock_enable(SPIx)		{	if (SPIx == SPI1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);} \
											else if (SPIx == SPI2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);} \
											else if (SPIx == SPI3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);} \
										}

#define	__spi_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
										else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
										else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
										else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
										else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
										else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
										else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
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

#if defined (GD32F10X_MD) || defined (GD32F10X_HD)

#define	__spi_clock_enable(spix)		{	if (spix == SPI0)		{rcu_periph_clock_enable(RCU_SPI0);} \
											else if (spix == SPI1)	{rcu_periph_clock_enable(RCU_SPI1);} \
											else if (spix == SPI2)	{rcu_periph_clock_enable(RCU_SPI2);} \
										}

#define	__spi_io_clock_enable(port)	{	if (port == GPIOA)		{rcu_periph_clock_enable(RCU_GPIOA);} \
										else if (port == GPIOB)	{rcu_periph_clock_enable(RCU_GPIOB);} \
										else if (port == GPIOC)	{rcu_periph_clock_enable(RCU_GPIOC);} \
										else if (port == GPIOD)	{rcu_periph_clock_enable(RCU_GPIOD);} \
										else if (port == GPIOE)	{rcu_periph_clock_enable(RCU_GPIOE);} \
										else if (port == GPIOF)	{rcu_periph_clock_enable(RCU_GPIOF);} \
										else if (port == GPIOG)	{rcu_periph_clock_enable(RCU_GPIOG);} \
									}
								
#define	__spi_config_io_af_pp(port, pin)    gpio_init(port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, pin);	
													
#define	__spi_config_io_out_pp(port, pin)	gpio_init(port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin);

#define	__spi_config_io_in_pu(port, pin)	gpio_init(port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pin);
                                                
#define	__spi_config_io_in_pn(port, pin)	gpio_init(port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, pin);

#define __spi_io_write(port, pin, value)	gpio_bit_write(port, pin, (bit_status)value);

#define __spi_get_prescaler(prescaler)	(	prescaler == 2 ? SPI_PSC_2 : \
											prescaler == 4 ? SPI_PSC_4 : \
											prescaler == 8 ? SPI_PSC_8 : \
											prescaler == 16 ? SPI_PSC_16 : \
											prescaler == 32 ? SPI_PSC_32 : \
											prescaler == 64 ? SPI_PSC_64 : \
											prescaler == 128 ? SPI_PSC_128 : \
											prescaler == 256 ? SPI_PSC_256 : \
											(int)0	)
#endif					
										
/* 引脚配置层 */
static void __spi_cs_write(spi_hard_dev_t *dev, uint8_t bit_val);

/* 协议层 */
static void __spi_start(spi_hard_dev_t *dev);
static void __spi_stop(spi_hard_dev_t *dev);
static uint8_t __spi_swap_byte(spi_hard_dev_t *dev, uint8_t send_byte);
static int8_t __spi_deinit(spi_hard_dev_t *dev);
										
/******************************************************************************
 * @brief	初始化硬件SPI
 * @param	dev	:	spi_hard_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t spi_hard_init(spi_hard_dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 开启时钟 */
	__spi_clock_enable(dev->config.spix);
	__spi_io_clock_enable(dev->config.sck_port);
	__spi_io_clock_enable(dev->config.mosi_port);
	__spi_io_clock_enable(dev->config.miso_port);
	__spi_io_clock_enable(dev->config.cs_port);
	
	/* 配置GPIO */
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD) || defined (GD32F10X_MD) || defined (GD32F10X_HD)
	__spi_config_io_af_pp(dev->config.sck_port, dev->config.sck_pin);
	__spi_config_io_af_pp(dev->config.mosi_port, dev->config.mosi_pin);
	__spi_config_io_in_pu(dev->config.miso_port, dev->config.miso_pin);
	__spi_config_io_out_pp(dev->config.cs_port, dev->config.cs_pin);
	
	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	__spi_config_io_af_pp(dev->config.sck_port, dev->config.sck_pin);
	__spi_config_io_af_pp(dev->config.mosi_port, dev->config.mosi_pin);
	__spi_config_io_af_pp(dev->config.miso_port, dev->config.miso_pin);
	__spi_config_io_out_pp(dev->config.cs_port, dev->config.cs_pin);
	
	/* STM32F4配置为复用输出时需要配置引脚复用映射 */
	GPIO_PinAFConfig(	dev->config.sck_port, 
						__spi_get_gpio_pin_sourse(dev->config.sck_pin), 
						__spi_get_gpio_af(dev->config.spix)	);
						
	GPIO_PinAFConfig(	dev->config.mosi_port, 
						__spi_get_gpio_pin_sourse(dev->config.mosi_pin), 
						__spi_get_gpio_af(dev->config.spix)	);
						
	GPIO_PinAFConfig(	dev->config.miso_port, 
						__spi_get_gpio_pin_sourse(dev->config.miso_pin), 
						__spi_get_gpio_af(dev->config.spix)	);
	
	#endif
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD) || defined(STM32F40_41xxx) || \
		defined(STM32F411xE) || defined(STM32F429_439xx)

	/* 配置硬件SPI */
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;											// 主机
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;						// 双线全双工
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;										// 8位数据帧
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;										// 高位先行
	SPI_InitStructure.SPI_BaudRatePrescaler = __spi_get_prescaler(dev->config.prescaler);	// 分频系数
	if(dev->config.mode == SPI_MODE_0)						// 模式0
	{	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	}
	else if(dev->config.mode == SPI_MODE_1)					// 模式1
	{	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	}
	else if(dev->config.mode == SPI_MODE_2)					// 模式2
	{	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	}
	else if(dev->config.mode == SPI_MODE_3)					// 模式3
	{	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	}
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				// 软件NSS
	SPI_InitStructure.SPI_CRCPolynomial = 7;				// CRC，不使用
	SPI_Init(dev->config.spix, &SPI_InitStructure);
	
	/* 开启硬件SPI */
	SPI_Cmd(dev->config.spix, ENABLE);

	#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)

	/* 配置硬件SPI */
	spi_i2s_deinit(dev->config.spix);

	spi_parameter_struct spi_struct;
	spi_struct.device_mode = SPI_MASTER;
	spi_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;	// 全双工
	spi_struct.frame_size = SPI_FRAMESIZE_8BIT;
	spi_struct.nss = SPI_NSS_SOFT;
	spi_struct.endian = SPI_ENDIAN_MSB;
	if (dev->config.mode == SPI_MODE_0)			spi_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	else if (dev->config.mode == SPI_MODE_1)	spi_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_1EDGE;
	else if (dev->config.mode == SPI_MODE_2)	spi_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
	else if (dev->config.mode == SPI_MODE_3)	spi_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
	spi_struct.prescale = __spi_get_prescaler(dev->config.prescaler);
	spi_init(dev->config.spix, &spi_struct);

	/* 开启硬件SPI */
	spi_enable(dev->config.spix);

	#endif
	
	/* 函数指针赋值 */
	dev->cs_write = __spi_cs_write;
	dev->start = __spi_start;
	dev->stop = __spi_stop;
	dev->swap_byte = __spi_swap_byte;
	dev->deinit = __spi_deinit;
	
	dev->init_flag = true;

	/* 设置默认电平 */
	__spi_cs_write(dev, 1);
	
	return 0;
}

/******************************************************************************
 * @brief	硬件SPI写CS引脚电平
 * @param	dev		:	spi_hard_dev_t 结构体指针
 * @param	bit_val	:	协议层传入的当前需要写入CS的电平，范围0~1
 * @return	无
 ******************************************************************************/
static void __spi_cs_write(spi_hard_dev_t *dev, uint8_t bit_val)
{
	__spi_io_write(dev->config.cs_port, dev->config.cs_pin, bit_val);
}

/******************************************************************************
 * @brief	硬件SPI起始
 * @param	dev	:	spi_hard_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_start(spi_hard_dev_t *dev)
{
	__spi_cs_write(dev, 0);
}

/******************************************************************************
 * @brief	硬件SPI停止
 * @param	dev	:	spi_hard_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __spi_stop(spi_hard_dev_t *dev)
{
	__spi_cs_write(dev, 1);
}

/******************************************************************************
 * @brief	硬件SPI交换一个字节
 * @param	dev			:	spi_hard_dev_t 结构体指针
 * @param	send_byte	:	发送的字节
 * @return	接收的字节
 ******************************************************************************/
static uint8_t __spi_swap_byte(spi_hard_dev_t *dev, uint8_t send_byte)
{
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD) || \
		defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	
	while(SPI_I2S_GetFlagStatus(dev->config.spix, SPI_I2S_FLAG_TXE) != SET);		// 等待TXE置1，表示发送寄存器为空，发送一个字节
	SPI_I2S_SendData(dev->config.spix, send_byte);								// 发送字节
	while(SPI_I2S_GetFlagStatus(dev->config.spix, SPI_I2S_FLAG_RXNE) != SET);		// 等待RXNE置1，表示接收寄存器非空，收到一个字节
	
	return SPI_I2S_ReceiveData(dev->config.spix);	// 读取接收到的字节
	
	#elif defined (GD32F10X_MD) || defined (GD32F10X_HD)

	while (spi_i2s_flag_get(dev->config.spix, SPI_FLAG_TBE) != 1);	// 等待TBE置1，表示发送寄存器为空，发送一个字节
	spi_i2s_data_transmit(dev->config.spix, send_byte);				// 发送字节
	while (spi_i2s_flag_get(dev->config.spix, SPI_FLAG_RBNE) != 1);	// 等待RBNE置1，表示接收寄存器非空，收到一个字节
	return spi_i2s_data_receive(dev->config.spix);					// 读取接收到的字节
	
	#endif
}

/******************************************************************************
 * @brief	去初始化硬件SPI
 * @param	dev	:	spi_hard_dev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __spi_deinit(spi_hard_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	return 0;
}
