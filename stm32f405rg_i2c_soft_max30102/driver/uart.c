#include "uart.h"

/*************************** STM32F1系列 ***************************/
#if defined(STM32F10X_MD) || defined(STM32F10X_HD)

#define	__uart_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
											else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
											else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
											else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
											else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
											else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
											else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
										}

#define	__uart_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__uart_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

	/*************************** STM32F1系列：UART1、2、3可用 ***************************/
	#if defined(STM32F10X_MD)

	#define	__uart_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
											else if (uartx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
											else if (uartx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);} \
										}

	#define	__uart_dma_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if (uartx == USART2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if (uartx == USART3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
											}

	#define __uart_get_irqn(uartx)	(	uartx == USART1 ? USART1_IRQn : \
										uartx == USART2 ? USART2_IRQn : \
										uartx == USART3 ? USART3_IRQn : \
										(int)0	)

	#define	__uart_get_dma_channel(uartx)	(	uartx == USART1 ? DMA1_Channel5 : \
												uartx == USART2 ? DMA1_Channel6 : \
												uartx == USART3 ? DMA1_Channel3 : \
												(int)0)
    
	/*************************** STM32F1系列：UART1、2、3、4可用 ***************************/											
	#elif defined(STM32F10X_HD)
										
	#define	__uart_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
											else if (uartx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
											else if (uartx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);} \
											else if (uartx == UART4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);} \
										}

	#define	__uart_dma_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if (uartx == USART2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if (uartx == USART3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if (uartx == UART4)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);} \
											}

	#define __uart_get_irqn(uartx)	(	uartx == USART1 ? USART1_IRQn : \
										uartx == USART2 ? USART2_IRQn : \
										uartx == USART3 ? USART3_IRQn : \
										uartx == UART4 ? UART4_IRQn : \
										(int)0	)

	#define	__uart_get_dma_channel(uartx)	(	uartx == USART1 ? DMA1_Channel5 : \
												uartx == USART2 ? DMA1_Channel6 : \
												uartx == USART3 ? DMA1_Channel3 : \
												uartx == UART4 ? DMA2_Channel3 : \
												(int)0)
	#endif

#endif

/*************************** STM32F4系列 ***************************/
#if defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)

#define	__uart_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
											else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
											else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
											else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
											else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
											else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
											else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
										}

#define	__uart_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __uart_get_gpio_pin_sourse(pin)	(	pin == GPIO_Pin_0 ? GPIO_PinSource0 : \
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
											(int)0)

	/*************************** STM32F4系列：UART1、2、3、4、5、6可用 ***************************/
	#if defined(STM32F40_41xxx) || defined(STM32F429_439xx)

	#define	__uart_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
											else if (uartx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
											else if (uartx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);} \
											else if (uartx == UART4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);} \
											else if (uartx == UART5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);} \
											else if (uartx == USART6)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);} \
										}

	#define	__uart_dma_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
												else if (uartx == USART2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
												else if (uartx == USART3)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
												else if (uartx == UART4)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
												else if (uartx == UART5)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
												else if (uartx == USART6)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
											}

	#define __uart_get_irqn(uartx)	(	uartx == USART1 ? USART1_IRQn : \
										uartx == USART2 ? USART2_IRQn : \
										uartx == USART3 ? USART3_IRQn : \
										uartx == UART4 ? UART4_IRQn : \
										uartx == UART5 ? UART5_IRQn : \
										uartx == USART6 ? USART6_IRQn : \
										(int)0	)

	#define __uart_get_dma_stream(uartx)	(	uartx == USART1 ? DMA2_Stream5 : \
												uartx == USART2 ? DMA1_Stream5 : \
												uartx == USART3 ? DMA1_Stream1 : \
												uartx == UART4 ? DMA1_Stream2 : \
												uartx == UART5 ? DMA1_Stream0 : \
												uartx == USART6 ? DMA2_Stream1 : \
												(int)0)

	#define	__uart_get_dma_channel(uartx)	(	uartx == USART1 ? DMA_Channel_4 : \
												uartx == USART2 ? DMA_Channel_4 : \
												uartx == USART3 ? DMA_Channel_4 : \
												uartx == UART4 ? DMA_Channel_4 : \
												uartx == UART5 ? DMA_Channel_4 : \
												uartx == USART6 ? DMA_Channel_5 : \
												(int)0)
	
	/*************************** STM32F4系列：UART1、2、6可用 ***************************/
	#elif defined(STM32F411xE)

	#define	__uart_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
											else if (uartx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
											else if (uartx == USART6)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);} \
										}

	#define	__uart_dma_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
												else if (uartx == USART2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
												else if (uartx == USART6)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
											}

	#define __uart_get_irqn(uartx)	(	uartx == USART1 ? USART1_IRQn : \
										uartx == USART2 ? USART2_IRQn : \
										uartx == USART6 ? USART6_IRQn : \
										(int)0	)

	#define __uart_get_dma_stream(uartx)	(	uartx == USART1 ? DMA2_Stream5 : \
												uartx == USART2 ? DMA1_Stream5 : \
												uartx == USART6 ? DMA2_Stream1 : \
												(int)0)

	#define	__uart_get_dma_channel(uartx)	(	uartx == USART1 ? DMA_Channel_4 : \
												uartx == USART2 ? DMA_Channel_4 : \
												uartx == USART6 ? DMA_Channel_5 : \
												(int)0)
	#endif

#endif

/* 缓冲区与接收控制块定义 */
#if UART1_ENABLE
uint8_t uart1_tx_buf[UART1_TX_SIZE];
uint8_t uart1_rx_buf[UART1_RX_SIZE];
uart_rx_cb_t uart1_rx_cb;
#endif
#if UART2_ENABLE
uint8_t uart2_tx_buf[UART2_TX_SIZE];
uint8_t uart2_rx_buf[UART2_RX_SIZE];
uart_rx_cb_t uart2_rx_cb;
#endif
#if UART3_ENABLE
uint8_t uart3_tx_buf[UART3_TX_SIZE];
uint8_t uart3_rx_buf[UART3_RX_SIZE];
uart_rx_cb_t uart3_rx_cb;
#endif
#if UART4_ENABLE
uint8_t uart4_tx_buf[UART4_TX_SIZE];
uint8_t uart4_rx_buf[UART4_RX_SIZE];
uart_rx_cb_t uart4_rx_cb;
#endif
#if UART5_ENABLE
uint8_t uart5_tx_buf[UART5_TX_SIZE];
uint8_t uart5_rx_buf[UART5_RX_SIZE];
uart_rx_cb_t uart5_rx_cb;
#endif
#if UART6_ENABLE
uint8_t uart6_tx_buf[UART6_TX_SIZE];
uint8_t uart6_rx_buf[UART6_RX_SIZE];
uart_rx_cb_t uart6_rx_cb;
#endif

/* 函数声明 */
static void __uart_dma_init(uart_dev_t *dev);
static void __uart_control_block_init(uart_dev_t *dev);
#if UART1_ENABLE
static void __uart1_printf(char *format, ...);
static void __uart1_send(uint8_t *data, uint32_t len);
static char *__uart1_recv(void);
#endif
#if UART2_ENABLE
static void __uart2_printf(char *format, ...);
static void __uart2_send(uint8_t *data, uint32_t len);
static char *__uart2_recv(void);
#endif
#if UART3_ENABLE
static void __uart3_printf(char *format, ...);
static void __uart3_send(uint8_t *data, uint32_t len);
static char *__uart3_recv(void);
#endif
#if UART4_ENABLE
static void __uart4_printf(char *format, ...);
static void __uart4_send(uint8_t *data, uint32_t len);
static char *__uart4_recv(void);
#endif
#if UART5_ENABLE
static void __uart5_printf(char *format, ...);
static void __uart5_send(uint8_t *data, uint32_t len);
static char *__uart5_recv(void);
#endif
#if UART6_ENABLE
static void __uart6_printf(char *format, ...);
static void __uart6_send(uint8_t *data, uint32_t len);
static char *__uart6_recv(void);
#endif
static int8_t __uart_deinit(uart_dev_t *dev);

/******************************************************************************
 * @brief	初始化UART，只能配置支持DMA的串口
 * @param	dev	:  uart_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int8_t uart_init(uart_dev_t *dev)
{
	if (!dev)
		return -1;

	/* 开启时钟 */
	__uart_clock_enable(dev->config.uartx);
	__uart_io_clock_enable(dev->config.tx_port);
	__uart_io_clock_enable(dev->config.rx_port);

	/* 配置GPIO */
    #if defined(STM32F10X_MD) || defined(STM32F10X_HD)
	__uart_config_io_af_pp(dev->config.tx_port, dev->config.tx_pin);	// 串口发送配置为复用推挽输出
	__uart_config_io_in_pu(dev->config.rx_port, dev->config.rx_pin);	// 串口接收配置为上拉输入
	
	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	__uart_config_io_af_pp(dev->config.tx_port, dev->config.tx_pin);	// 串口发送配置为复用模式
    __uart_config_io_af_pp(dev->config.rx_port, dev->config.rx_pin);	// 串口接收配置为复用模式
	if (dev->config.uartx == USART1)
	{	GPIO_PinAFConfig(dev->config.tx_port, __uart_get_gpio_pin_sourse(dev->config.tx_pin), GPIO_AF_USART1);
		GPIO_PinAFConfig(dev->config.rx_port, __uart_get_gpio_pin_sourse(dev->config.rx_pin), GPIO_AF_USART1);	}
	else if (dev->config.uartx == USART2)
	{	GPIO_PinAFConfig(dev->config.tx_port, __uart_get_gpio_pin_sourse(dev->config.tx_pin), GPIO_AF_USART2);
		GPIO_PinAFConfig(dev->config.rx_port, __uart_get_gpio_pin_sourse(dev->config.rx_pin), GPIO_AF_USART2);	}
	else if (dev->config.uartx == USART3)
	{	GPIO_PinAFConfig(dev->config.tx_port, __uart_get_gpio_pin_sourse(dev->config.tx_pin), GPIO_AF_USART3);
		GPIO_PinAFConfig(dev->config.rx_port, __uart_get_gpio_pin_sourse(dev->config.rx_pin), GPIO_AF_USART3);	}
	else if (dev->config.uartx == UART4)
	{	GPIO_PinAFConfig(dev->config.tx_port, __uart_get_gpio_pin_sourse(dev->config.tx_pin), GPIO_AF_UART4);
		GPIO_PinAFConfig(dev->config.rx_port, __uart_get_gpio_pin_sourse(dev->config.rx_pin), GPIO_AF_UART4);	}
	else if (dev->config.uartx == UART5)
	{	GPIO_PinAFConfig(dev->config.tx_port, __uart_get_gpio_pin_sourse(dev->config.tx_pin), GPIO_AF_UART5);
		GPIO_PinAFConfig(dev->config.rx_port, __uart_get_gpio_pin_sourse(dev->config.rx_pin), GPIO_AF_UART5);	}
	else if (dev->config.uartx == USART6)
	{	GPIO_PinAFConfig(dev->config.tx_port, __uart_get_gpio_pin_sourse(dev->config.tx_pin), GPIO_AF_USART6);
		GPIO_PinAFConfig(dev->config.rx_port, __uart_get_gpio_pin_sourse(dev->config.rx_pin), GPIO_AF_USART6);	}
	#endif
	
	/* 配置USART */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = dev->config.baud;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(dev->config.uartx, &USART_InitStructure);
	
	/* 配置中断 */
	USART_ITConfig(dev->config.uartx, USART_IT_IDLE, ENABLE);	// 使能UART空闲中断
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = __uart_get_irqn(dev->config.uartx);
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	/* 配置DMA */
	__uart_dma_init(dev);

	/* UART控制块初始化 */
	__uart_control_block_init(dev);
	
	/* 开启USART */
	USART_Cmd(dev->config.uartx, ENABLE);
	
	/* 函数指针赋值 */
	#if UART1_ENABLE
	if (dev->config.uartx == USART1){dev->printf = __uart1_printf;	dev->send = __uart1_send;	dev->recv = __uart1_recv;}
	#endif
	#if UART2_ENABLE
	if (dev->config.uartx == USART2){dev->printf = __uart2_printf;	dev->send = __uart2_send;	dev->recv = __uart2_recv;}
	#endif
	#if UART3_ENABLE
	if (dev->config.uartx == USART3){dev->printf = __uart3_printf;	dev->send = __uart3_send;	dev->recv = __uart3_recv;}
	#endif
	#if UART4_ENABLE
	if (dev->config.uartx == UART4)	{dev->printf = __uart4_printf;	dev->send = __uart4_send;	dev->recv = __uart4_recv;}
	#endif
	#if UART5_ENABLE
	if (dev->config.uartx == UART5)	{dev->printf = __uart5_printf;	dev->send = __uart5_send;	dev->recv = __uart5_recv;}
	#endif
	#if UART6_ENABLE
	if (dev->config.uartx == USART6){dev->printf = __uart6_printf;	dev->send = __uart6_send;	dev->recv = __uart6_recv;}
	#endif

	dev->deinit = __uart_deinit;
	
	dev->init_flag = true;

	return 0;
}

/******************************************************************************
 * @brief	UART配置为DMA接收数据
 * @param	dev	:  uart_dev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
static void __uart_dma_init(uart_dev_t *dev)
{
	/* 开启DMA时钟 */
	__uart_dma_clock_enable(dev->config.uartx);

	/* 配置DMA */
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	DMA_DeInit(__uart_get_dma_channel(dev->config.uartx));							// 将DMA的通道寄存器重设为缺省值
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dev->config.uartx->DR;	// 外设基地址
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			// 外设数据宽度为8位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				// 外设地址寄存器不变
	/* DMA内存基地址 */
	#if UART1_ENABLE
	if (dev->config.uartx == USART1)	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart1_rx_buf;
	#endif
	#if UART2_ENABLE
	if (dev->config.uartx == USART2)	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart2_rx_buf;
	#endif
	#if UART3_ENABLE
	if (dev->config.uartx == USART3)	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart3_rx_buf;
	#endif
	#if UART4_ENABLE
	if (dev->config.uartx == UART4)		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart4_rx_buf;
	#endif
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					// 内存数据宽度为8位
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							// 内存地址寄存器递增

	/* 指定DMA缓冲区大小为单次最大接收量加1，只有空闲中断才能判断接收完成 */
	#if UART1_ENABLE
	if (dev->config.uartx == USART1)	DMA_InitStructure.DMA_BufferSize = UART1_RX_MAX + 1;
	#endif
	#if UART2_ENABLE
	if (dev->config.uartx == USART2)	DMA_InitStructure.DMA_BufferSize = UART2_RX_MAX + 1;
	#endif
	#if UART3_ENABLE
	if (dev->config.uartx == USART3)	DMA_InitStructure.DMA_BufferSize = UART3_RX_MAX + 1;
	#endif
	#if UART4_ENABLE
	if (dev->config.uartx == UART4)		DMA_InitStructure.DMA_BufferSize = UART4_RX_MAX + 1;
	#endif
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						// 数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// 工作在正常模式，一次传输后自动结束
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;							// 没有设置为内存到内存传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						// 高优先级 
	DMA_Init(__uart_get_dma_channel(dev->config.uartx), &DMA_InitStructure);

	/* 开启DMA */
	DMA_Cmd(__uart_get_dma_channel(dev->config.uartx), ENABLE);

	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

	DMA_DeInit(__uart_get_dma_stream(dev->config.uartx));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = __uart_get_dma_channel(dev->config.uartx);			// 选择DMA通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dev->config.uartx->DR;		// DMA外设基地址
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;				// 外设数据单位8位
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					// 外设地址不增

	/* DMA内存基地址 */
	#if UART1_ENABLE
	if (dev->config.uartx == USART1)	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart1_rx_buf;
	#endif
	#if UART2_ENABLE
	if (dev->config.uartx == USART2)	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart2_rx_buf;
	#endif
	#if UART3_ENABLE
	if (dev->config.uartx == USART3)	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart3_rx_buf;
	#endif
	#if UART4_ENABLE
	if (dev->config.uartx == UART4)		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart4_rx_buf;
	#endif
	#if UART5_ENABLE
	if (dev->config.uartx == UART5)		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart5_rx_buf;
	#endif
	#if UART6_ENABLE
	if (dev->config.uartx == USART6)	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart6_rx_buf;
	#endif
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						// 内存数据单位8位
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								// 内存地址自增

	/* 指定DMA缓冲区大小为单次最大接收量加1，只有空闲中断才能判断接收完成 */
	#if UART1_ENABLE
	if (dev->config.uartx == USART1)	DMA_InitStructure.DMA_BufferSize = UART1_RX_MAX + 1;
	#endif
	#if UART2_ENABLE
	if (dev->config.uartx == USART2)	DMA_InitStructure.DMA_BufferSize = UART2_RX_MAX + 1;
	#endif
	#if UART3_ENABLE
	if (dev->config.uartx == USART3)	DMA_InitStructure.DMA_BufferSize = UART3_RX_MAX + 1;
	#endif
	#if UART4_ENABLE
	if (dev->config.uartx == UART4)		DMA_InitStructure.DMA_BufferSize = UART4_RX_MAX + 1;
	#endif
	#if UART5_ENABLE
	if (dev->config.uartx == UART5)		DMA_InitStructure.DMA_BufferSize = UART5_RX_MAX + 1;
	#endif 
	#if UART6_ENABLE
	if (dev->config.uartx == USART6)	DMA_InitStructure.DMA_BufferSize = UART6_RX_MAX + 1;
	#endif
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								// 从外设读取发送到内存
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										// 工作在正常模式，一次传输后自动结束
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;									// 优先级：高
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;								// 禁用FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;						// FIFO阈值为满
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;					// 外设突发传输为单次
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;							// 内存突发传输为单次
    DMA_Init(__uart_get_dma_stream(dev->config.uartx), &DMA_InitStructure);

	/* 开启DMA */
    DMA_Cmd(__uart_get_dma_stream(dev->config.uartx), ENABLE);

	#endif

	/* 启用UART的DMA请求 */
	USART_DMACmd(dev->config.uartx, USART_DMAReq_Rx, ENABLE);
}

/******************************************************************************
 * @brief	UART控制块初始化
 * @param	无
 * @return	无
 ******************************************************************************/
static void __uart_control_block_init(uart_dev_t *dev)
{
	#if UART1_ENABLE
	if (dev->config.uartx == USART1)
	{
		/* 初始化索引指针指向索引数组的第0位 */
		uart1_rx_cb.index_in = &uart1_rx_cb.index_buf[0];
		uart1_rx_cb.index_out = &uart1_rx_cb.index_buf[0];

		/* 标记索引数组的end */
		uart1_rx_cb.index_end = &uart1_rx_cb.index_buf[INDEX_BUF_NUM - 1];

		/* 标记第一段数据的start为接收数据的第0位 */
		uart1_rx_cb.index_in->start = &uart1_rx_buf[0];

		uart1_rx_cb.data_cnt = 0;
	}
	#endif

	#if UART2_ENABLE
	if (dev->config.uartx == USART2)
	{
		/* 初始化索引指针指向索引数组的第0位 */
		uart2_rx_cb.index_in = &uart2_rx_cb.index_buf[0];
		uart2_rx_cb.index_out = &uart2_rx_cb.index_buf[0];

		/* 标记索引数组的end */
		uart2_rx_cb.index_end = &uart2_rx_cb.index_buf[INDEX_BUF_NUM - 1];

		/* 标记第一段数据的start为接收数据的第0位 */
		uart2_rx_cb.index_in->start = &uart2_rx_buf[0];
	}
	#endif

	#if UART3_ENABLE
	if (dev->config.uartx == USART3)
	{
		/* 初始化索引指针指向索引数组的第0位 */
		uart3_rx_cb.index_in = &uart3_rx_cb.index_buf[0];
		uart3_rx_cb.index_out = &uart3_rx_cb.index_buf[0];

		/* 标记索引数组的end */
		uart3_rx_cb.index_end = &uart3_rx_cb.index_buf[INDEX_BUF_NUM - 1];

		/* 标记第一段数据的start为接收数据的第0位 */
		uart3_rx_cb.index_in->start = &uart3_rx_buf[0];
	}
	#endif

	#if UART4_ENABLE
	if (dev->config.uartx == UART4)
	{
		/* 初始化索引指针指向索引数组的第0位 */
		uart4_rx_cb.index_in = &uart4_rx_cb.index_buf[0];
		uart4_rx_cb.index_out = &uart4_rx_cb.index_buf[0];

		/* 标记索引数组的end */
		uart4_rx_cb.index_end = &uart4_rx_cb.index_buf[INDEX_BUF_NUM - 1];

		/* 标记第一段数据的start为接收数据的第0位 */
		uart4_rx_cb.index_in->start = &uart4_rx_buf[0];
	}
	#endif

	#if UART5_ENABLE
	if (dev->config.uartx == UART5)
	{
		/* 初始化索引指针指向索引数组的第0位 */
		uart5_rx_cb.index_in = &uart5_rx_cb.index_buf[0];
		uart5_rx_cb.index_out = &uart5_rx_cb.index_buf[0];

		/* 标记索引数组的end */
		uart5_rx_cb.index_end = &uart5_rx_cb.index_buf[INDEX_BUF_NUM - 1];

		/* 标记第一段数据的start为接收数据的第0位 */
		uart5_rx_cb.index_in->start = &uart5_rx_buf[0];
	}
	#endif

	#if UART6_ENABLE
	if (dev->config.uartx == USART6)
	{
		/* 初始化索引指针指向索引数组的第0位 */
		uart6_rx_cb.index_in = &uart6_rx_cb.index_buf[0];
		uart6_rx_cb.index_out = &uart6_rx_cb.index_buf[0];

		/* 标记索引数组的end */
		uart6_rx_cb.index_end = &uart6_rx_cb.index_buf[INDEX_BUF_NUM - 1];

		/* 标记第一段数据的start为接收数据的第0位 */
		uart6_rx_cb.index_in->start = &uart6_rx_buf[0];
	}
	#endif
}

#if UART1_ENABLE
/******************************************************************************
 * @brief	串口1打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart1_printf(char *format, ...)
{
	uint16_t i;
	va_list list_data;

	va_start(list_data, format);

	vsprintf((char *)uart1_tx_buf, format, list_data);
	va_end(list_data);
	for (i = 0; i < strlen((const char *)uart1_tx_buf); i++)
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) != 1);
		USART_SendData(USART1, uart1_tx_buf[i]);
	}
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != 1);
}
#endif

#if UART2_ENABLE
/******************************************************************************
 * @brief	串口2打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart2_printf(char *format, ...)
{
	uint16_t i;
	va_list list_data;

	va_start(list_data, format);

	vsprintf((char *)uart2_tx_buf, format, list_data);
	va_end(list_data);
	for (i = 0; i < strlen((const char *)uart2_tx_buf); i++)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != 1);
		USART_SendData(USART2, uart2_tx_buf[i]);
	}
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) != 1);
}
#endif

#if UART3_ENABLE
/******************************************************************************
 * @brief	串口3打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart3_printf(char *format, ...)
{
	uint16_t i;
	va_list list_data;

	va_start(list_data, format);
	
	vsprintf((char *)uart3_tx_buf, format, list_data);
	va_end(list_data);
	for (i = 0; i < strlen((const char *)uart3_tx_buf); i++)
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) != 1);
		USART_SendData(USART3, uart3_tx_buf[i]);
	}
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) != 1);
}
#endif

#if UART4_ENABLE
/******************************************************************************
 * @brief	串口4打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart4_printf(char *format, ...)
{
	uint16_t i;
	va_list list_data;

	va_start(list_data, format);

	vsprintf((char *)uart4_tx_buf, format, list_data);
	va_end(list_data);
	for (i = 0; i < strlen((const char *)uart4_tx_buf); i++)
	{
		while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) != 1);
		USART_SendData(UART4, uart4_tx_buf[i]);
	}
	while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != 1);
}
#endif

#if UART5_ENABLE
/******************************************************************************
 * @brief	串口5打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart5_printf(char *format, ...)
{
	uint16_t i;
	va_list list_data;

	va_start(list_data, format);
	
	vsprintf((char *)uart5_tx_buf, format, list_data);
	va_end(list_data);
	for (i = 0; i < strlen((const char *)uart5_tx_buf); i++)
	{
		while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) != 1);
		USART_SendData(UART5, uart5_tx_buf[i]);
	}
	while (USART_GetFlagStatus(UART5, USART_FLAG_TC) != 1);
}
#endif

#if UART6_ENABLE
/******************************************************************************
 * @brief	串口6打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart6_printf(char *format, ...)
{
	uint16_t i;
	va_list list_data;

	va_start(list_data, format);

	vsprintf((char *)uart6_tx_buf, format, list_data);
	va_end(list_data);
	for (i = 0; i < strlen((const char *)uart6_tx_buf); i++)
	{
		while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) != 1);
		USART_SendData(USART6, uart6_tx_buf[i]);
	}
	while (USART_GetFlagStatus(USART6, USART_FLAG_TC) != 1);
}
#endif

#if UART1_ENABLE
/******************************************************************************
 * @brief	UART1发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart1_send(uint8_t *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(USART1, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	// 等待发送完成
	}
}
#endif

#if UART2_ENABLE
/******************************************************************************
 * @brief	UART2发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart2_send(uint8_t *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(USART2, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	// 等待发送完成
	}
}
#endif

#if UART3_ENABLE
/******************************************************************************
 * @brief	UART3发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart3_send(uint8_t *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(USART3, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	// 等待发送完成
	}
}
#endif

#if UART4_ENABLE
/******************************************************************************
 * @brief	UART4发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart4_send(uint8_t *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(UART4, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);	// 等待发送完成
	}
}
#endif

#if UART5_ENABLE
/******************************************************************************
 * @brief	UART5发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart5_send(uint8_t *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(UART5, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	// 等待发送完成
	}
}
#endif

#if UART6_ENABLE
/******************************************************************************
 * @brief	UART6发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart6_send(uint8_t *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(USART6, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);	// 等待发送完成
	}
}
#endif

#if UART1_ENABLE
/******************************************************************************
 * @brief	UART1接收
 * @param	无
 * @return	无
 ******************************************************************************/
static char *__uart1_recv(void)
{
	char *ret_str;

	/* 缓冲区中有未处理数据 */
	if (uart1_rx_cb.index_in != uart1_rx_cb.index_out)
	{
		/* 末尾加上'\0'，确保可以作为字符串处理 */
        *(uart1_rx_cb.index_out->end + 1) = '\0';

        ret_str = (char *)uart1_rx_cb.index_out->start;

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		uart1_rx_cb.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (uart1_rx_cb.index_out == uart1_rx_cb.index_end)
		{
			uart1_rx_cb.index_out = &uart1_rx_cb.index_buf[0];
		}

		return ret_str;
	}

	return NULL;
}
#endif

#if UART2_ENABLE
/******************************************************************************
 * @brief	UART2接收
 * @param	无
 * @return	无
 ******************************************************************************/
static char *__uart2_recv(void)
{
	char *ret_str;
	
	/* 缓冲区中有未处理数据 */
	if (uart2_rx_cb.index_in != uart2_rx_cb.index_out)
	{
		/* 末尾加上'\0'，确保可以作为字符串处理 */
        *(uart2_rx_cb.index_out->end + 1) = '\0';

        ret_str = (char *)uart2_rx_cb.index_out->start;

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		uart2_rx_cb.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (uart2_rx_cb.index_out == uart2_rx_cb.index_end)
		{
			uart2_rx_cb.index_out = &uart2_rx_cb.index_buf[0];
		}

		return ret_str;
	}

	return NULL;
}
#endif

#if UART3_ENABLE
/******************************************************************************
 * @brief	UART3接收
 * @param	无
 * @return	无
 ******************************************************************************/
static char *__uart3_recv(void)
{
	char *ret_str;

	/* 缓冲区中有未处理数据 */
	if (uart3_rx_cb.index_in != uart3_rx_cb.index_out)
	{
		/* 末尾加上'\0'，确保可以作为字符串处理 */
        *(uart3_rx_cb.index_out->end + 1) = '\0';

        ret_str = (char *)uart3_rx_cb.index_out->start;

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		uart3_rx_cb.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (uart3_rx_cb.index_out == uart3_rx_cb.index_end)
		{
			uart3_rx_cb.index_out = &uart3_rx_cb.index_buf[0];
		}

		return ret_str;
	}

	return NULL;
}
#endif

#if UART4_ENABLE
/******************************************************************************
 * @brief	UART4接收
 * @param	无
 * @return	无
 ******************************************************************************/
static char *__uart4_recv(void)
{
	char *ret_str;

	/* 缓冲区中有未处理数据 */
	if (uart4_rx_cb.index_in != uart4_rx_cb.index_out)
	{
		/* 末尾加上'\0'，确保可以作为字符串处理 */
        *(uart4_rx_cb.index_out->end + 1) = '\0';

        ret_str = (char *)uart4_rx_cb.index_out->start;

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		uart4_rx_cb.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (uart4_rx_cb.index_out == uart4_rx_cb.index_end)
		{
			uart4_rx_cb.index_out = &uart4_rx_cb.index_buf[0];
		}

		return ret_str;
	}

	return NULL;
}
#endif

#if UART5_ENABLE
/******************************************************************************
 * @brief	UART5接收
 * @param	无
 * @return	无
 ******************************************************************************/
static char *__uart5_recv(void)
{
	char *ret_str;

	/* 缓冲区中有未处理数据 */
	if (uart5_rx_cb.index_in != uart5_rx_cb.index_out)
	{
		/* 末尾加上'\0'，确保可以作为字符串处理 */
        *(uart5_rx_cb.index_out->end + 1) = '\0';

        ret_str = (char *)uart5_rx_cb.index_out->start;

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		uart5_rx_cb.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (uart5_rx_cb.index_out == uart5_rx_cb.index_end)
		{
			uart5_rx_cb.index_out = &uart5_rx_cb.index_buf[0];
		}

		return ret_str;
	}

	return NULL;
}
#endif

#if UART6_ENABLE
/******************************************************************************
 * @brief	UART6接收
 * @param	无
 * @return	无
 ******************************************************************************/
static char *__uart6_recv(void)
{
	char *ret_str;
	
	/* 缓冲区中有未处理数据 */
	if (uart6_rx_cb.index_in != uart6_rx_cb.index_out)
	{
		/* 末尾加上'\0'，确保可以作为字符串处理 */
        *(uart6_rx_cb.index_out->end + 1) = '\0';

        ret_str = (char *)uart6_rx_cb.index_out->start;

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		uart6_rx_cb.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (uart6_rx_cb.index_out == uart6_rx_cb.index_end)
		{
			uart6_rx_cb.index_out = &uart6_rx_cb.index_buf[0];
		}

		return ret_str;
	}

	return NULL;
}
#endif

/******************************************************************************
 * @brief	去初始化UART
 * @param	dev   :  uart_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __uart_deinit(uart_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;
	
	return 0;
}

#if UART1_ENABLE
/******************************************************************************
 * @brief	UART1中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART1_IRQHandler(void)
{
	volatile uint8_t clear;
	
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = USART1->SR;
		clear = USART1->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		uart1_rx_cb.data_cnt += ((UART1_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Channel5));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		uart1_rx_cb.data_cnt += ((UART1_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA2_Stream5));

		#endif

		/* 标记这一段数据的end */
		uart1_rx_cb.index_in->end = &uart1_rx_buf[uart1_rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		uart1_rx_cb.index_in++;
		if (uart1_rx_cb.index_in == uart1_rx_cb.index_end)
		{
			uart1_rx_cb.index_in = &uart1_rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (UART1_RX_SIZE - uart1_rx_cb.data_cnt >= UART1_RX_MAX)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			uart1_rx_cb.index_in->start = &uart1_rx_buf[uart1_rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			uart1_rx_cb.index_in->start = &uart1_rx_buf[0];
			uart1_rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel5, DISABLE);								// 关闭DMA
		while(((DMA1_Channel5->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel5->CNDTR = UART1_RX_MAX + 1;						// 设置数据长度
		DMA1_Channel5->CMAR = (uint32_t)(uart1_rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel5, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA2_Stream5, DISABLE);									// 关闭DMA
		while ((DMA2_Stream5->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA2_Stream5->NDTR = UART1_RX_MAX + 1;                          // 设置数据长度
		DMA2_Stream5->M0AR = (uint32_t)(uart1_rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA2_Stream5, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#if UART2_ENABLE
/******************************************************************************
 * @brief	UART2中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART2_IRQHandler(void)
{
	volatile uint8_t clear;

	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = USART2->SR;
		clear = USART2->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		uart2_rx_cb.data_cnt += ((UART2_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Channel6));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		uart2_rx_cb.data_cnt += ((UART2_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Stream5));

		#endif

		/* 标记这一段数据的end */
		uart2_rx_cb.index_in->end = &uart2_rx_buf[uart2_rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		uart2_rx_cb.index_in++;
		if (uart2_rx_cb.index_in == uart2_rx_cb.index_end)
		{
			uart2_rx_cb.index_in = &uart2_rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (UART2_RX_SIZE - uart2_rx_cb.data_cnt >= UART2_RX_MAX)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			uart2_rx_cb.index_in->start = &uart2_rx_buf[uart2_rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			uart2_rx_cb.index_in->start = &uart2_rx_buf[0];
			uart2_rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel6, DISABLE);								// 关闭DMA
		while(((DMA1_Channel6->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel6->CNDTR = UART2_RX_MAX + 1;						// 设置数据长度
		DMA1_Channel6->CMAR = (uint32_t)(uart2_rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel6, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA1_Stream5, DISABLE);									// 关闭DMA
		while ((DMA1_Stream5->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA1_Stream5->NDTR = UART2_RX_MAX + 1;                          // 设置数据长度
		DMA1_Stream5->M0AR = (uint32_t)(uart2_rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA1_Stream5, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#if UART3_ENABLE
/******************************************************************************
 * @brief	UART3中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART3_IRQHandler(void)
{
	volatile uint8_t clear;

	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = USART3->SR;
		clear = USART3->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		uart3_rx_cb.data_cnt += ((UART3_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Channel3));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		uart3_rx_cb.data_cnt += ((UART3_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Stream1));

		#endif

		/* 标记这一段数据的end */
		uart3_rx_cb.index_in->end = &uart3_rx_buf[uart3_rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		uart3_rx_cb.index_in++;
		if (uart3_rx_cb.index_in == uart3_rx_cb.index_end)
		{
			uart3_rx_cb.index_in = &uart3_rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (UART3_RX_SIZE - uart3_rx_cb.data_cnt >= UART3_RX_MAX)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			uart3_rx_cb.index_in->start = &uart3_rx_buf[uart3_rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			uart3_rx_cb.index_in->start = &uart3_rx_buf[0];
			uart3_rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel3, DISABLE);								// 关闭DMA
		while(((DMA1_Channel3->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel3->CNDTR = UART3_RX_MAX + 1;						// 设置数据长度
		DMA1_Channel3->CMAR = (uint32_t)(uart3_rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel3, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA1_Stream1, DISABLE);									// 关闭DMA
		while ((DMA1_Stream1->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA1_Stream1->NDTR = UART3_RX_MAX + 1;                          // 设置数据长度
		DMA1_Stream1->M0AR = (uint32_t)(uart3_rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA1_Stream1, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#if UART4_ENABLE
/******************************************************************************
 * @brief	UART4中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void UART4_IRQHandler(void)
{
	volatile uint8_t clear;

	if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = UART4->SR;
		clear = UART4->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		uart4_rx_cb.data_cnt += ((UART4_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA2_Channel3));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		uart4_rx_cb.data_cnt += ((UART4_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Stream2));

		#endif

		/* 标记这一段数据的end */
		uart4_rx_cb.index_in->end = &uart4_rx_buf[uart4_rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		uart4_rx_cb.index_in++;
		if (uart4_rx_cb.index_in == uart4_rx_cb.index_end)
		{
			uart4_rx_cb.index_in = &uart4_rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (UART4_RX_SIZE - uart4_rx_cb.data_cnt >= UART4_RX_MAX)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			uart4_rx_cb.index_in->start = &uart4_rx_buf[uart4_rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			uart4_rx_cb.index_in->start = &uart4_rx_buf[0];
			uart4_rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA2_Channel3, DISABLE);								// 关闭DMA
		while(((DMA2_Channel3->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA2_Channel3->CNDTR = UART4_RX_MAX + 1;						// 设置数据长度
		DMA2_Channel3->CMAR = (uint32_t)(uart4_rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA2_Channel3, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA1_Stream2, DISABLE);									// 关闭DMA
		while ((DMA1_Stream2->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA1_Stream2->NDTR = UART4_RX_MAX + 1;                          // 设置数据长度
		DMA1_Stream2->M0AR = (uint32_t)(uart4_rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA1_Stream2, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#if UART5_ENABLE
/******************************************************************************
* @brief	UART5中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void UART5_IRQHandler(void)
{
	volatile uint8_t clear;

	if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = UART5->SR;
		clear = UART5->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		uart5_rx_cb.data_cnt += ((UART5_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Channel3));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		uart5_rx_cb.data_cnt += ((UART5_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Stream0));

		#endif

		/* 标记这一段数据的end */
		uart5_rx_cb.index_in->end = &uart5_rx_buf[uart5_rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		uart5_rx_cb.index_in++;
		if (uart5_rx_cb.index_in == uart5_rx_cb.index_end)
		{
			uart5_rx_cb.index_in = &uart5_rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (UART5_RX_SIZE - uart5_rx_cb.data_cnt >= UART5_RX_MAX)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			uart5_rx_cb.index_in->start = &uart5_rx_buf[uart5_rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			uart5_rx_cb.index_in->start = &uart5_rx_buf[0];
			uart5_rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel3, DISABLE);								// 关闭DMA
		while(((DMA1_Channel3->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel3->CNDTR = UART5_RX_MAX + 1;						// 设置数据长度
		DMA1_Channel3->CMAR = (uint32_t)(uart5_rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel3, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA1_Stream0, DISABLE);									// 关闭DMA
		while ((DMA1_Stream0->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA1_Stream0->NDTR = UART5_RX_MAX + 1;                          // 设置数据长度
		DMA1_Stream0->M0AR = (uint32_t)(uart5_rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA1_Stream0, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#if UART6_ENABLE
/******************************************************************************
* @brief	UART6中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART6_IRQHandler(void)
{
	volatile uint8_t clear;

	if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = USART6->SR;
		clear = USART6->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		uart6_rx_cb.data_cnt += ((UART6_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA1_Channel3));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		uart6_rx_cb.data_cnt += ((UART6_RX_MAX + 1) - DMA_GetCurrDataCounter(DMA2_Stream1));

		#endif

		/* 标记这一段数据的end */
		uart6_rx_cb.index_in->end = &uart6_rx_buf[uart6_rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		uart6_rx_cb.index_in++;
		if (uart6_rx_cb.index_in == uart6_rx_cb.index_end)
		{
			uart6_rx_cb.index_in = &uart6_rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (UART6_RX_SIZE - uart6_rx_cb.data_cnt >= UART6_RX_MAX)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			uart6_rx_cb.index_in->start = &uart6_rx_buf[uart6_rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			uart6_rx_cb.index_in->start = &uart6_rx_buf[0];
			uart6_rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel3, DISABLE);								// 关闭DMA
		while(((DMA1_Channel3->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel3->CNDTR = UART6_RX_MAX + 1;						// 设置数据长度
		DMA1_Channel3->CMAR = (uint32_t)(uart6_rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel3, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA2_Stream1, DISABLE);									// 关闭DMA
		while ((DMA2_Stream1->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA2_Stream1->NDTR = UART6_RX_MAX + 1;                          // 设置数据长度
		DMA2_Stream1->M0AR = (uint32_t)(uart6_rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA2_Stream1, ENABLE);									// 开启DMA
		#endif
	}
}
#endif
