#include "uart.h"

#ifdef USE_STDPERIPH_DRIVER

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

	/*************************** STM32F1_MD系列：UART1、2、3可用 ***************************/
	#if defined(STM32F10X_MD)

	#define UART1_AVAILABLE
	#define UART2_AVAILABLE
	#define UART3_AVAILABLE

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

	#define	__uart_get_index(uartx)	(	uartx == USART1 ? 0 : \
										uartx == USART2 ? 1 : \
										uartx == USART3 ? 2 : \
										(int)-1)
    
	/*************************** STM32F1_HD系列：UART1、2、3、4可用（UART5无DMA） ***************************/											
	#elif defined(STM32F10X_HD)

	#define UART1_AVAILABLE
	#define UART2_AVAILABLE
	#define UART3_AVAILABLE
	#define UART4_AVAILABLE
										
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

	#define	__uart_get_index(uartx)	(	uartx == USART1 ? 0 : \
										uartx == USART2 ? 1 : \
										uartx == USART3 ? 2 : \
										uartx == UART4 ? 3 : \
										(int)-1)
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

	#define UART1_AVAILABLE
	#define UART2_AVAILABLE
	#define UART3_AVAILABLE
	#define UART4_AVAILABLE
	#define UART5_AVAILABLE
	#define UART6_AVAILABLE

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

	#define	__uart_get_index(uartx)	(	uartx == USART1 ? 0 : \
										uartx == USART2 ? 1 : \
										uartx == USART3 ? 2 : \
										uartx == UART4 ? 3 : \
										uartx == UART5 ? 4 : \
										uartx == USART6 ? 5 : \
										(int)-1)
	
	/*************************** STM32F4系列：UART1、2、6可用 ***************************/
	#elif defined(STM32F411xE)

	#define UART1_AVAILABLE
	#define UART2_AVAILABLE
	#define UART6_AVAILABLE

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

	#define	__uart_get_index(uartx)	(	uartx == USART1 ? 0 : \
										uartx == USART2 ? 1 : \
										uartx == USART6 ? 5 : \
										(int)-1)
	#endif

#endif

#endif

/* 最大串口数量 */
#define MAX_UART_NUM 6

/* 已注册串口设备指针数组 */
static uart_dev_t *uart_registry[MAX_UART_NUM] = {NULL};

/* 函数声明 */
static void __uart_dma_init(uart_dev_t *dev);
static void __uart_printf(uart_dev_t *dev, const char *format, va_list args);
static void __uart_send(uart_dev_t *dev, uint8_t *data, uint32_t len);
static char *__uart_recv(uart_dev_t *dev);
static int8_t __uart_deinit(uart_dev_t *dev);

#ifdef UART1_AVAILABLE
static void __uart1_printf(char *format, ...);
static void __uart1_send(uint8_t *data, uint32_t len);
static char *__uart1_recv(void);
#endif

#ifdef UART2_AVAILABLE
static void __uart2_printf(char *format, ...);
static void __uart2_send(uint8_t *data, uint32_t len);
static char *__uart2_recv(void);
#endif

#ifdef UART3_AVAILABLE
static void __uart3_printf(char *format, ...);
static void __uart3_send(uint8_t *data, uint32_t len);
static char *__uart3_recv(void);
#endif

#ifdef UART4_AVAILABLE
static void __uart4_printf(char *format, ...);
static void __uart4_send(uint8_t *data, uint32_t len);
static char *__uart4_recv(void);
#endif

#ifdef UART5_AVAILABLE
static void __uart5_printf(char *format, ...);
static void __uart5_send(uint8_t *data, uint32_t len);
static char *__uart5_recv(void);
#endif

#ifdef UART6_AVAILABLE
static void __uart6_printf(char *format, ...);
static void __uart6_send(uint8_t *data, uint32_t len);
static char *__uart6_recv(void);
#endif

/******************************************************************************
 * @brief	初始化UART，只能配置支持DMA的串口
 * @param	dev	:  uart_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int8_t uart_init(uart_dev_t *dev)
{
	if (!dev)
		return -1;

	int8_t index;
	
	/* 注册串口设备 */
	index = __uart_get_index(dev->config.uartx);
	if (index >= 0)
	{
		uart_registry[index] = dev;
	}
    
	/* 初始化接收数据控制块 */
	dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];
	dev->rx_cb.index_out = &dev->rx_cb.index_buf[0];
	dev->rx_cb.index_end = &dev->rx_cb.index_buf[INDEX_BUF_NUM - 1];
	dev->rx_cb.index_in->start = dev->config.rx_buf;
	dev->rx_cb.data_cnt = 0;

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
	
	/* 开启USART */
	USART_Cmd(dev->config.uartx, ENABLE);
	
	/* 函数指针赋值 */
	#ifdef UART1_AVAILABLE
	if (dev->config.uartx == USART1){dev->printf = __uart1_printf;	dev->send = __uart1_send;	dev->recv = __uart1_recv;}
	#endif

	#ifdef UART2_AVAILABLE
	if (dev->config.uartx == USART2){dev->printf = __uart2_printf;	dev->send = __uart2_send;	dev->recv = __uart2_recv;}
	#endif

	#ifdef UART3_AVAILABLE
	if (dev->config.uartx == USART3){dev->printf = __uart3_printf;	dev->send = __uart3_send;	dev->recv = __uart3_recv;}
	#endif

	#ifdef UART4_AVAILABLE
	if (dev->config.uartx == UART4)	{dev->printf = __uart4_printf;	dev->send = __uart4_send;	dev->recv = __uart4_recv;}
	#endif

	#ifdef UART5_AVAILABLE
	if (dev->config.uartx == UART5)	{dev->printf = __uart5_printf;	dev->send = __uart5_send;	dev->recv = __uart5_recv;}
	#endif

	#ifdef UART6_AVAILABLE
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
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dev->config.rx_buf;			// DMA内存基地址
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					// 内存数据宽度为8位
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							// 内存地址寄存器递增
	DMA_InitStructure.DMA_BufferSize = dev->config.rx_single_max + 1;				// 指定DMA缓冲区大小为单次最大接收量加1，只有空闲中断才能判断接收完成
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;								// 数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;									// 工作在正常模式，一次传输后自动结束
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;									// 没有设置为内存到内存传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;								// 高优先级 
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
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dev->config.rx_buf;				// DMA内存基地址
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						// 内存数据单位8位
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								// 内存地址自增
	DMA_InitStructure.DMA_BufferSize = dev->config.rx_single_max + 1;					// 指定DMA缓冲区大小为单次最大接收量加1，只有空闲中断才能判断接收完成
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
 * @brief	UART通用打印函数，内部使用
 * @param	uartx	:	uart_dev_t 结构体指针
 * @param	format	:	格式化参数
 * @param	args	:	va_list
 * @return	无
 ******************************************************************************/
static void __uart_printf(uart_dev_t *dev, const char *format, va_list args)
{
	uint16_t i;
	vsnprintf((char *)dev->config.tx_buf, dev->config.tx_buf_size, format, args);
	for (i = 0; i < strlen((const char *)dev->config.tx_buf); i++)
	{
		while (USART_GetFlagStatus(dev->config.uartx, USART_FLAG_TXE) != 1);
		USART_SendData(dev->config.uartx, dev->config.tx_buf[i]);
	}
	while (USART_GetFlagStatus(dev->config.uartx, USART_FLAG_TC) != 1);
}

/******************************************************************************
 * @brief	UART通用发送数据函数，内部使用
 * @param	dev		:	uart_dev_t 结构体指针
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart_send(uart_dev_t *dev, uint8_t *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		USART_SendData(dev->config.uartx, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (USART_GetFlagStatus(dev->config.uartx, USART_FLAG_TXE) == RESET);	// 等待发送完成
	}
}

/******************************************************************************
 * @brief	UART通用接收函数，内部使用
 * @param	dev	:	uart_dev_t 结构体指针
 * @return	接收到数据字符串的首地址，未接收到数据则为NULL
 ******************************************************************************/
static char *__uart_recv(uart_dev_t *dev)
{
	char *ret_str;

	/* 缓冲区中有未处理数据 */
	if (dev->rx_cb.index_in != dev->rx_cb.index_out)
	{
		/* 末尾加上'\0'，确保可以作为字符串处理 */
        *(dev->rx_cb.index_out->end + 1) = '\0';

        ret_str = (char *)dev->rx_cb.index_out->start;

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		dev->rx_cb.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (dev->rx_cb.index_out == dev->rx_cb.index_end)
		{
			dev->rx_cb.index_out = &dev->rx_cb.index_buf[0];
		}

		return ret_str;
	}

	return NULL;
}

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

#ifdef UART1_AVAILABLE
/******************************************************************************
 * @brief	UART1打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart1_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART1)];
	if (!dev) return;

	va_list args;
	va_start(args, format);
	__uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/******************************************************************************
 * @brief	UART1发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart1_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART1)];
	if (!dev) return;

	__uart_send(dev, data, len);
}

/******************************************************************************
 * @brief	UART1接收
 * @param	无
 * @return	接收到数据字符串的首地址，未接收到数据则为NULL
 ******************************************************************************/
static char *__uart1_recv(void)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART1)];

	return __uart_recv(dev);
}
#endif

#ifdef UART2_AVAILABLE
/******************************************************************************
 * @brief	UART2打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart2_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART2)];
	if (!dev) return;

	va_list args;
	va_start(args, format);
	__uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/******************************************************************************
 * @brief	UART2发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart2_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART2)];
	if (!dev) return;

	__uart_send(dev, data, len);
}

/******************************************************************************
 * @brief	UART2接收
 * @param	无
 * @return	接收到数据字符串的首地址，未接收到数据则为NULL
 ******************************************************************************/
static char *__uart2_recv(void)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART2)];
	
	return __uart_recv(dev);
}
#endif

#ifdef UART3_AVAILABLE
/******************************************************************************
 * @brief	UART3打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart3_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART3)];
	if (!dev) return;

	va_list args;
	va_start(args, format);
	__uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/******************************************************************************
 * @brief	UART3发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart3_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART3)];
	if (!dev) return;

	__uart_send(dev, data, len);
}

/******************************************************************************
 * @brief	UART3接收
 * @param	无
 * @return	接收到数据字符串的首地址，未接收到数据则为NULL
 ******************************************************************************/
static char *__uart3_recv(void)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART3)];

	return __uart_recv(dev);
}
#endif

#ifdef UART4_AVAILABLE
/******************************************************************************
 * @brief	UART4打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart4_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(UART4)];
	if (!dev) return;

	va_list args;
	va_start(args, format);
	__uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/******************************************************************************
 * @brief	UART4发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart4_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(UART4)];
	if (!dev) return;

	__uart_send(dev, data, len);
}

/******************************************************************************
 * @brief	UART4接收
 * @param	无
 * @return	接收到数据字符串的首地址，未接收到数据则为NULL
 ******************************************************************************/
static char *__uart4_recv(void)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(UART4)];

	return __uart_recv(dev);
}
#endif

#ifdef UART5_AVAILABLE
/******************************************************************************
 * @brief	UART5打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart5_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(UART5)];
	if (!dev) return;

	va_list args;
	va_start(args, format);
	__uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/******************************************************************************
 * @brief	UART5发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart5_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(UART5)];
	if (!dev) return;

	__uart_send(dev, data, len);
}

/******************************************************************************
 * @brief	UART5接收
 * @param	无
 * @return	接收到数据字符串的首地址，未接收到数据则为NULL
 ******************************************************************************/
static char *__uart5_recv(void)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(UART5)];

	return __uart_recv(dev);
}
#endif

#ifdef UART6_AVAILABLE
/******************************************************************************
 * @brief	UART6打印
 * @param	format
 * @return	无
 ******************************************************************************/
static void __uart6_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART6)];
	if (!dev) return;

	va_list args;
	va_start(args, format);
	__uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/******************************************************************************
 * @brief	UART6发送数据
 * @param	data	:	数据
 * @param	len		:	数据长度
 * @return	无
 ******************************************************************************/
static void __uart6_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART6)];
	if (!dev) return;

	__uart_send(dev, data, len);
}

/******************************************************************************
 * @brief	UART6接收
 * @param	无
 * @return	接收到数据字符串的首地址，未接收到数据则为NULL
 ******************************************************************************/
static char *__uart6_recv(void)
{
	uart_dev_t *dev = uart_registry[__uart_get_index(USART6)];

	return __uart_recv(dev);
}
#endif

#ifdef UART1_AVAILABLE
/******************************************************************************
 * @brief	UART1中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART1_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registry[__uart_get_index(USART1)];		// 根据索引找到已注册的串口设备
	if (!dev) return;
	
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = USART1->SR;
		clear = USART1->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Channel5));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA2_Stream5));

		#endif

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
		{
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel5, DISABLE);								// 关闭DMA
		while(((DMA1_Channel5->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel5->CNDTR = dev->config.rx_single_max + 1;			// 设置数据长度
		DMA1_Channel5->CMAR = (uint32_t)(dev->rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel5, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA2_Stream5, DISABLE);									// 关闭DMA
		while ((DMA2_Stream5->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA2_Stream5->NDTR = dev->config.rx_single_max + 1;				// 设置数据长度
		DMA2_Stream5->M0AR = (uint32_t)(dev->rx_cb.index_in->start);   	// 设置内存地址
		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA2_Stream5, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#ifdef UART2_AVAILABLE
/******************************************************************************
 * @brief	UART2中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART2_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registry[__uart_get_index(USART2)];		// 根据索引找到已注册的串口设备
	if (!dev) return;

	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = USART2->SR;
		clear = USART2->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Channel6));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Stream5));

		#endif

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
		{
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel6, DISABLE);								// 关闭DMA
		while(((DMA1_Channel6->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel6->CNDTR = dev->config.rx_single_max + 1;						// 设置数据长度
		DMA1_Channel6->CMAR = (uint32_t)(dev->rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel6, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA1_Stream5, DISABLE);									// 关闭DMA
		while ((DMA1_Stream5->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA1_Stream5->NDTR = dev->config.rx_single_max + 1;                          // 设置数据长度
		DMA1_Stream5->M0AR = (uint32_t)(dev->rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA1_Stream5, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#ifdef UART3_AVAILABLE
/******************************************************************************
 * @brief	UART3中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART3_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registry[__uart_get_index(USART3)];		// 根据索引找到已注册的串口设备
	if (!dev) return;

	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = USART3->SR;
		clear = USART3->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Channel3));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Stream1));

		#endif

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
		{
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel3, DISABLE);								// 关闭DMA
		while(((DMA1_Channel3->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel3->CNDTR = dev->config.rx_single_max + 1;						// 设置数据长度
		DMA1_Channel3->CMAR = (uint32_t)(dev->rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel3, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA1_Stream1, DISABLE);									// 关闭DMA
		while ((DMA1_Stream1->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA1_Stream1->NDTR = dev->config.rx_single_max + 1;                          // 设置数据长度
		DMA1_Stream1->M0AR = (uint32_t)(dev->rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA1_Stream1, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#ifdef UART4_AVAILABLE
/******************************************************************************
 * @brief	UART4中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void UART4_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registry[__uart_get_index(UART4)];		// 根据索引找到已注册的串口设备
	if (!dev) return;

	if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = UART4->SR;
		clear = UART4->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA2_Channel3));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Stream2));

		#endif

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
		{
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA2_Channel3, DISABLE);								// 关闭DMA
		while(((DMA2_Channel3->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA2_Channel3->CNDTR = dev->config.rx_single_max + 1;						// 设置数据长度
		DMA2_Channel3->CMAR = (uint32_t)(dev->rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA2_Channel3, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA1_Stream2, DISABLE);									// 关闭DMA
		while ((DMA1_Stream2->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA1_Stream2->NDTR = dev->config.rx_single_max + 1;                          // 设置数据长度
		DMA1_Stream2->M0AR = (uint32_t)(dev->rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA1_Stream2, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#ifdef UART5_AVAILABLE
/******************************************************************************
 * @brief	UART5中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void UART5_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registry[__uart_get_index(UART5)];		// 根据索引找到已注册的串口设备
	if (!dev) return;

	if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = UART5->SR;
		clear = UART5->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Channel3));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Stream0));

		#endif

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
		{
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel3, DISABLE);								// 关闭DMA
		while(((DMA1_Channel3->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel3->CNDTR = dev->config.rx_single_max + 1;						// 设置数据长度
		DMA1_Channel3->CMAR = (uint32_t)(dev->rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel3, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA1_Stream0, DISABLE);									// 关闭DMA
		while ((DMA1_Stream0->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA1_Stream0->NDTR = dev->config.rx_single_max + 1;                          // 设置数据长度
		DMA1_Stream0->M0AR = (uint32_t)(dev->rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA1_Stream0, ENABLE);									// 开启DMA
		#endif
	}
}
#endif

#ifdef UART6_AVAILABLE
/******************************************************************************
 * @brief	UART6中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART6_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registry[__uart_get_index(USART6)];		// 根据索引找到已注册的串口设备
	if (!dev) return;

	if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)   // 空闲中断
	{
		/* 清除空闲中断 */
		clear = USART6->SR;
		clear = USART6->DR;
	
		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA1_Channel3));

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - DMA_GetCurrDataCounter(DMA2_Stream1));

		#endif

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
		{
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
		DMA_Cmd(DMA1_Channel3, DISABLE);								// 关闭DMA
		while(((DMA1_Channel3->CCR) & DMA_CCR1_EN) == 1);				// 等待DMA真正关闭
		DMA1_Channel3->CNDTR = dev->config.rx_single_max + 1;						// 设置数据长度
		DMA1_Channel3->CMAR = (uint32_t)(dev->rx_cb.index_in->start);	// 设置内存地址
		DMA_Cmd(DMA1_Channel3, ENABLE);									// 开启DMA

		#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
		DMA_Cmd(DMA2_Stream1, DISABLE);									// 关闭DMA
		while ((DMA2_Stream1->CR & DMA_SxCR_EN) != 0);                  // 等待DMA真正关闭
		DMA2_Stream1->NDTR = dev->config.rx_single_max + 1;                          // 设置数据长度
		DMA2_Stream1->M0AR = (uint32_t)(dev->rx_cb.index_in->start);   // 设置内存地址
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);					// 清除DMA传输完成中断标志位
		DMA_Cmd(DMA2_Stream1, ENABLE);									// 开启DMA
		#endif
	}
}
#endif
