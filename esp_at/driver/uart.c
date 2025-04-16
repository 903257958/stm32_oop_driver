#include "uart.h"

#if defined(STM32F10X_MD)
											
#define __uart_get_irqn(uartx)	(	uartx == USART1 ? USART1_IRQn : \
									uartx == USART2 ? USART2_IRQn : \
									uartx == USART3 ? USART3_IRQn : \
									(int)0	)
											
#define __uart_get_index(uartx)	(	uartx == USART1 ? 0 : \
									uartx == USART2 ? 1 : \
									uartx == USART3 ? 2 : \
										(int)-1	)
										
#define	__uart_config_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
												else if (uartx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
												else if (uartx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);} \
											}

#define	__uart_config_gpio_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												}

#define	__uart_config_dma_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if (uartx == USART2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if (uartx == USART3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												}

#define	__uart_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__uart_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__uart_get_dma_channel(uartx)		(	uartx == USART1 ? DMA1_Channel5 : \
												uartx == USART2 ? DMA1_Channel6 : \
												uartx == USART3 ? DMA1_Channel3 : \
												(int)0)
                                                
#elif defined(STM32F10X_HD)
											
#define __uart_get_irqn(uartx)	(	uartx == USART1 ? USART1_IRQn : \
									uartx == USART2 ? USART2_IRQn : \
									uartx == USART3 ? USART3_IRQn : \
									uartx == UART4 ? UART4_IRQn : \
									uartx == UART5 ? UART5_IRQn : \
									(int)0	)
											
#define __uart_get_index(uartx)	(	uartx == USART1 ? 0 : \
									uartx == USART2 ? 1 : \
									uartx == USART3 ? 2 : \
									uartx == UART4 ? 3 : \
									uartx == UART5 ? 4 : \
										(int)-1	)
										
#define	__uart_config_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
												else if (uartx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
												else if (uartx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);} \
												else if (uartx == UART4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);} \
												else if (uartx == UART5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);} \
											}

#define	__uart_config_gpio_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												}

#define	__uart_config_dma_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if (uartx == USART2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if (uartx == USART3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if (uartx == UART4)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);} \
												}

#define	__uart_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__uart_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__uart_get_dma_channel(uartx)		(	uartx == USART1 ? DMA1_Channel5 : \
												uartx == USART2 ? DMA1_Channel6 : \
												uartx == USART3 ? DMA1_Channel3 : \
												uartx == UART4 ? DMA2_Channel3 : \
												(int)0)

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
											
#define __uart_get_irqn(uartx)	(	uartx == USART1 ? USART1_IRQn : \
									uartx == USART2 ? USART2_IRQn : \
									uartx == USART3 ? USART3_IRQn : \
									uartx == UART4 ? UART4_IRQn : \
									uartx == UART5 ? UART5_IRQn : \
									uartx == USART6 ? USART6_IRQn : \
									(int)0	)
											
#define __uart_get_index(uartx)	(	uartx == USART1 ? 0 : \
									uartx == USART2 ? 1 : \
									uartx == USART3 ? 2 : \
									uartx == UART4 ? 3 : \
									uartx == UART5 ? 4 : \
									uartx == USART6 ? 5 : \
									(int)-1	)
										
#define	__uart_config_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
												else if (uartx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
												else if (uartx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);} \
												else if (uartx == UART4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);} \
												else if (uartx == UART5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);} \
												else if (uartx == USART6)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);} \
											}

#define	__uart_config_gpio_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}

#define	__uart_config_dma_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
													else if (uartx == USART2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if (uartx == USART3)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if (uartx == UART4)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if (uartx == UART5)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if (uartx == USART6)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
												}

#define	__uart_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__uart_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __uart_get_dma_stream(uartx)	(	uartx == USART1 ? DMA2_Stream5 : \
											uartx == USART2 ? DMA1_Stream5 : \
											uartx == USART3 ? DMA1_Stream1 : \
											uartx == UART4 ? DMA1_Stream2 : \
											uartx == UART5 ? DMA1_Stream0 : \
											uartx == USART6 ? DMA2_Stream1 : \
											(int)0)

#define	__uart_get_dma_channel(uartx)		(	uartx == USART1 ? DMA_Channel_4 : \
												uartx == USART2 ? DMA_Channel_4 : \
												uartx == USART3 ? DMA_Channel_4 : \
												uartx == UART4 ? DMA_Channel_4 : \
												uartx == UART5 ? DMA_Channel_4 : \
												uartx == USART6 ? DMA_Channel_5 : \
												(int)0)
												
#define	__uart_get_dma_flag(uartx)		(	uartx == USART1 ? DMA_FLAG_TCIF5 : \
											uartx == USART2 ? DMA_FLAG_TCIF5 : \
											uartx == USART3 ? DMA_FLAG_TCIF1 : \
											uartx == UART4 ? DMA_FLAG_TCIF2 : \
											uartx == UART5 ? DMA_FLAG_TCIF0 : \
											uartx == USART6 ? DMA_FLAG_TCIF1 : \
											(int)0)

#elif defined(STM32F411xE)
											
#define __uart_get_irqn(uartx)	(	uartx == USART1 ? USART1_IRQn : \
									uartx == USART2 ? USART2_IRQn : \
                                    uartx == USART6 ? USART6_IRQn : \
									(int)0	)
											
#define __uart_get_index(uartx)	(	uartx == USART1 ? 0 : \
									uartx == USART2 ? 1 : \
                                    uartx == USART6 ? 5 : \
									(int)-1	)
										
#define	__uart_config_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
												else if (uartx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
                                                else if (uartx == USART6)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);} \
											}

#define	__uart_config_gpio_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}

#define	__uart_config_dma_clock_enable(uartx)	{	if (uartx == USART1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
													else if (uartx == USART2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
                                                    else if (uartx == USART6)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
												}

#define	__uart_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__uart_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __uart_get_dma_stream(uartx)	(	uartx == USART1 ? DMA2_Stream5 : \
											uartx == USART2 ? DMA1_Stream5 : \
                                            uartx == USART6 ? DMA2_Stream1 : \
											(int)0)

#define	__uart_get_dma_channel(uartx)		(	uartx == USART1 ? DMA_Channel_4 : \
												uartx == USART2 ? DMA_Channel_4 : \
                                                uartx == USART6 ? DMA_Channel_5 : \
												(int)0)
												
#define	__uart_get_dma_flag(uartx)		(	uartx == USART1 ? DMA_FLAG_TCIF5 : \
											uartx == USART2 ? DMA_FLAG_TCIF5 : \
                                            uartx == USART6 ? DMA_FLAG_TCIF1 : \
											(int)0)

#endif

#define MAX_USART_NUM			6									// 串口外设最大数量
#define MAX_RX_STRING_LENGTH	1024								// 接收文本数据包最大长度

static char g_rx_str[MAX_USART_NUM][MAX_RX_STRING_LENGTH];			// 接收文本数据包
static uint8_t g_rx_str_flag[MAX_USART_NUM];						// 接收文本数据包标志位

/* UART私有数据结构体 */
typedef struct {
	uint8_t index;	// 索引
}UARTPrivData_t;

/* 函数声明 */
static int __uart_dma_recv_enable(UARTDev_t *dev);
static int __uart_send_byte(UARTDev_t *dev, uint8_t byte);
static int __uart_send_array(UARTDev_t *dev, uint8_t *array, uint16_t length);
static int __uart_send_string(UARTDev_t *dev, char *str);
static int __uart_send_number(UARTDev_t *dev, uint32_t num, uint8_t length);
static int __uart_send_hex_packet(UARTDev_t *dev, uint8_t *packet, uint8_t length, uint8_t head, uint8_t end);
static int __uart_printf(UARTDev_t *dev, char *format, ...);
static char *__uart_recv_string(UARTDev_t *dev);
static uint8_t __uart_recv_string_flag(UARTDev_t *dev);
static int __uart_deinit(UARTDev_t *dev);

/******************************************************************************
 * @brief	初始化UART
 * @param	dev	:  UARTDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int uart_init(UARTDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 保存私有数据 */
	dev->priv_data = (UARTPrivData_t *)malloc(sizeof(UARTPrivData_t));
	if (!dev->priv_data)
		return -1;
	
	UARTPrivData_t *priv_data = (UARTPrivData_t *)dev->priv_data;

	priv_data->index = __uart_get_index(dev->config.uartx);

	/* 配置时钟与GPIO */
	__uart_config_clock_enable(dev->config.uartx);
	__uart_config_gpio_clock_enable(dev->config.tx_port);
	__uart_config_gpio_clock_enable(dev->config.rx_port);
	__uart_config_io_af_pp(dev->config.tx_port, dev->config.tx_pin);
	__uart_config_io_af_pp(dev->config.rx_port, dev->config.rx_pin);
	
	#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	if (dev->config.uartx == USART1)
	{
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	}
	else if (dev->config.uartx == USART2)
	{
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	}
	else if (dev->config.uartx == USART2)
	{
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	}
	else if (dev->config.uartx == UART4)
	{
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	}
	else if (dev->config.uartx == UART5)
	{
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	}
	else if (dev->config.uartx == USART6)
	{
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	}
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
	USART_ITConfig(dev->config.uartx, USART_IT_RXNE, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = __uart_get_irqn(dev->config.uartx);
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 开启USART */
	USART_Cmd(dev->config.uartx, ENABLE);
	
	/* 函数指针赋值 */
	dev->send_byte = __uart_send_byte;
	dev->send_array = __uart_send_array;
	dev->send_string = __uart_send_string;
	dev->send_number = __uart_send_number;
	dev->send_hex_packet = __uart_send_hex_packet;
	dev->printf = __uart_printf;
	dev->recv_string = __uart_recv_string;
	dev->recv_string_flag = __uart_recv_string_flag;
	dev->deinit = __uart_deinit;
	
	dev->dma_Flag = false;
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	初始化UART并配置为DMA接收数据
 * @param	dev	:  UARTDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int uart_dma_init(UARTDev_t *dev)
{
	/* UART初始化 */
	uart_init(dev);
	
	/* 保存私有数据 */
	UARTPrivData_t *priv_data = (UARTPrivData_t *)dev->priv_data;
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	/* 配置DMA */
	__uart_config_dma_clock_enable(dev->config.uartx);
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(__uart_get_dma_channel(dev->config.uartx));								// 将DMA的通道寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dev->config.uartx->DR;		// DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(g_rx_str[priv_data->index]);		// DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;									// 数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = sizeof(g_rx_str[priv_data->index]);				// DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					// 外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								// 内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;				// 数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						// 数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										// 工作在正常模式，一次传输后自动结束
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;								// 中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;										// 没有设置为内存到内存传输
	DMA_Init(__uart_get_dma_channel(dev->config.uartx), &DMA_InitStructure);

	/* 中断使能 */
	USART_ITConfig(dev->config.uartx, USART_IT_RXNE, DISABLE);							// 关闭串口接受中断
	USART_ITConfig(dev->config.uartx, USART_IT_IDLE, ENABLE);								// 使能UART空闲中断

	/* 开启DMA */
	DMA_Cmd(__uart_get_dma_channel(dev->config.uartx), ENABLE);

	/* 启用UART的DMA请求 */
	USART_DMACmd(dev->config.uartx, USART_DMAReq_Rx, ENABLE);

	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	
	/* 配置DMA */
	__uart_config_dma_clock_enable(dev->config.uartx);									// 开启DMA时钟
	DMA_DeInit(__uart_get_dma_stream(dev->config.uartx));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = __uart_get_dma_channel(dev->config.uartx);			// 选择DMA通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dev->config.uartx->DR;			// DMA外设基地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(g_rx_str[priv_data->index]);		// DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								// 从外设读取发送到内存
    DMA_InitStructure.DMA_BufferSize = sizeof(g_rx_str[priv_data->index]);				// DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					// 外设地址不增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								// 内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;				// 外设数据单位8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						// 内存数据单位8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										// 工作在正常模式，一次传输后自动结束
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;								// 优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;								// 禁用FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;						// FIFO阈值为满
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;							// 内存突发传输为单次
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;					// 外设突发传输为单次
    DMA_Init(__uart_get_dma_stream(dev->config.uartx), &DMA_InitStructure);
					
	/* 中断使能 */
	USART_ITConfig(dev->config.uartx, USART_IT_RXNE, DISABLE);							// 关闭串口接受中断
	USART_ITConfig(dev->config.uartx, USART_IT_IDLE, ENABLE);								// 使能UART空闲中断

	/* 开启DMA */
    DMA_Cmd(__uart_get_dma_stream(dev->config.uartx), ENABLE);

	/* 启用UART的DMA请求 */
	USART_DMACmd(dev->config.uartx, USART_DMAReq_Rx, ENABLE);
	
	#endif
	
	/* 函数指针赋值 */
	dev->dma_recv_enable = __uart_dma_recv_enable;
	
	dev->dma_Flag = true;
	
	return 0;
}

/******************************************************************************
 * @brief	UART开启一次DMA接收
 * @param	dev	:  UARTDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __uart_dma_recv_enable(UARTDev_t *dev)
{
	UARTPrivData_t *priv_data = (UARTPrivData_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag || !dev->dma_Flag)
		return -1;
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	/* 将接收的内存部分数据清0 */
	memset(g_rx_str[priv_data->index], 0, sizeof(g_rx_str[priv_data->index]));
	
	/* 重新设置传输数据长度 */
	DMA_SetCurrDataCounter(__uart_get_dma_channel(dev->config.uartx), sizeof(g_rx_str[priv_data->index]));

	/* 重新打开DMA */
	DMA_Cmd(__uart_get_dma_channel(dev->config.uartx), ENABLE);
	
	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	
	/* 将接收的内存部分数据清0 */
	memset(g_rx_str[priv_data->index], 0, sizeof(g_rx_str[priv_data->index]));

	/* 清除标志位 */
	DMA_ClearFlag(__uart_get_dma_stream(dev->config.uartx), __uart_get_dma_flag(dev->config.uartx));
	USART_ClearFlag(dev->config.uartx, USART_FLAG_IDLE);

	/* 重新设置传输数据长度 */
	DMA_SetCurrDataCounter(__uart_get_dma_stream(dev->config.uartx), sizeof(g_rx_str[priv_data->index]));

	/* 重新打开DMA */
	DMA_Cmd(__uart_get_dma_stream(dev->config.uartx), ENABLE);
	
	#endif

	return 0;
}

/******************************************************************************
 * @brief	UART发送一个字节
 * @param	dev	:  UARTDev_t 结构体指针
 * @param	byte	:  要发送的字节
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/	
static int __uart_send_byte(UARTDev_t *dev, uint8_t byte)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	USART_SendData(dev->config.uartx, byte);	// 写DR寄存器
	while (RESET == USART_GetFlagStatus(dev->config.uartx, USART_FLAG_TXE));	// 等待TXE置1，不需要手动清除标志位
	
	return 0;
}

/******************************************************************************
 * @brief	UART发送一个数组
 * @param	dev	:  UARTDev_t 结构体指针
 * @param	array	:  要发送的数组首地址
 * @param	length	:  要发送的数组长度
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __uart_send_array(UARTDev_t *dev, uint8_t *array, uint16_t length)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	while (length--)
    {
        __uart_send_byte(dev, *array++);
    }
	
	return 0;
}

/******************************************************************************
 * @brief	UART发送一个字符串
 * @param	dev	:  UARTDev_t 结构体指针
 * @param	str		:  要发送的字符串首地址
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __uart_send_string(UARTDev_t *dev, char *str)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	while (*str != '\0')
	{
		__uart_send_byte(dev, *str++);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	次方函数
 * @param	x
 * @param	y
 * @return	x^y
 ******************************************************************************/
static uint32_t pow(uint32_t x, uint8_t y)
{
	uint32_t res = 1;
	while (y--)
	{
		res *= x;
	}
	return res;
}

/******************************************************************************
 * @brief	UART发送一个数字
 * @param	dev	:  UARTDev_t 结构体指针
 * @param	num		:  要发送的数字
 * @param	length	:  要发送的数字长度
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __uart_send_number(UARTDev_t *dev, uint32_t num, uint8_t length)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	uint8_t i;
	for(i = 0;i < length;i++)
	{
		__uart_send_byte(dev, num / pow(10, length - i - 1) % 10 + '0');
	}
	
	return 0;
}

/******************************************************************************
 * @brief	UART发送HEX数据包
 * @param	dev	:  UARTDev_t 结构体指针
 * @param	packet	:  要发送的HEX数据包首地址
 * @param	length	:  要发送的HEX数据包长度
 * @param	packet	:  要发送的HEX数据包头
 * @param	length	:  要发送的HEX数据包尾
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __uart_send_hex_packet(UARTDev_t *dev, uint8_t *packet, uint8_t length, uint8_t head, uint8_t end)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	__uart_send_byte(dev, head);	//发送包头
	__uart_send_array(dev, packet, length);	//发送数据包
	__uart_send_byte(dev, end);	//发送包尾
	
	return 0;
}

/******************************************************************************
 * @brief	printf重定义（仅UART1可用）
 ******************************************************************************/
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, ch);	// 写DR寄存器
	while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE));	// 等待TXE置1，不需要手动清除标志位
	return ch;
}

/******************************************************************************
 * @brief	printf函数封装（多个串口都可使用）
 * @param	dev	:  UARTDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __uart_printf(UARTDev_t *dev, char *format, ...)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	char str[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(str, format, arg);
	va_end(arg);
	__uart_send_string(dev, str);
	
	return 0;
}

/******************************************************************************
 * @brief	返回UART接收的文本数据包的首地址
 * @param	dev	:  UARTDev_t 结构体指针
 * @return	UART接收的文本数据包的首地址
 ******************************************************************************/
static char *__uart_recv_string(UARTDev_t *dev)
{
	UARTPrivData_t *priv_data = dev->priv_data;
	
	return g_rx_str[priv_data->index];
}

/******************************************************************************
 * @brief	返回UART接收的文本数据包标志位（并自动清除）
 * @param	dev	:  UARTDev_t 结构体指针
 * @return	UART接收的文本数据包标志位
 ******************************************************************************/
static uint8_t __uart_recv_string_flag(UARTDev_t *dev)
{
	UARTPrivData_t *priv_data = dev->priv_data;
	
	if (g_rx_str_flag[priv_data->index] == 1)
	{
		g_rx_str_flag[priv_data->index] = 0;
		
		return 1;
	}
	return 0;
}

/******************************************************************************
 * @brief	去初始化UART
 * @param	dev   :  UARTDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __uart_deinit(UARTDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;
	
	dev->dma_Flag = false;	// 修改初始化标志
	dev->init_flag = false;
	
	return 0;
}

/******************************************************************************
 * @brief	UART接收文本数据包中断回调函数
 * @param	uartx	：	串口外设
 * @return	无
 ******************************************************************************/
static void __uart_recv_string_callback(UARTPER_t uartx)
{
	uint8_t index;
	if (uartx == USART1)		{index = 0;}
	else if (uartx == USART2)	{index = 1;}
	else if (uartx == USART3)	{index = 2;}
	else if (uartx == UART4)	{index = 3;}
	else if (uartx == UART5)	{index = 4;}
    #if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	else if (uartx == USART6)	{index = 5;}
    #endif
	
	static uint8_t rx_string_num = 0; 						// 接收到文本数据包的数据个数
	
	uint8_t rx_byte = USART_ReceiveData(uartx);
	
	if (rx_string_num < (uint8_t)(sizeof(g_rx_str[index]) - 1)) 	// 确保不越界
	{
		g_rx_str[index][rx_string_num++] = rx_byte;
		if (rx_byte == '\n' && rx_string_num > 1 && g_rx_str[index][rx_string_num - 2] == '\r')	// 检查是否接收到完整的数据包
		{
            g_rx_str[index][rx_string_num - 2] = '\0'; // 结束符
            rx_string_num = 0; 							// 重置接收计数器
            g_rx_str_flag[index] = 1; 					// 标记数据包已接收完成
		}
	}
	else
	{
		rx_string_num = 0;								// 如果接收到的数据超出了数组的大小，重置接收计数器
		g_rx_str_flag[index] = 0;
	}
}

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
			clear = USART1->SR;			// 清除空闲中断
			clear = USART1->DR;			// 清除空闲中断
			g_rx_str_flag[0] = 1;		// 置接收标志位
		
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
			DMA_Cmd(DMA1_Channel5, DISABLE);
			#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
			DMA_Cmd(DMA2_Stream5, DISABLE);
			#endif
	}
	
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		__uart_recv_string_callback(USART1);

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);	// 清除中断标志位
	}
}

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
			clear = USART2->SR;		// 清除空闲中断
			clear = USART2->DR;		// 清除空闲中断
			g_rx_str_flag[1] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
			DMA_Cmd(DMA1_Channel6, DISABLE);
			#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
			DMA_Cmd(DMA1_Stream5, DISABLE);
			#endif
	}

	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		__uart_recv_string_callback(USART2);
		
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);	// 清除中断标志位
	}
}

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
			clear = USART3->SR;		// 清除空闲中断
			clear = USART3->DR;		// 清除空闲中断
			g_rx_str_flag[2] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
			DMA_Cmd(DMA1_Channel3, DISABLE);
			#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
			DMA_Cmd(DMA1_Stream1, DISABLE);
			#endif
	}

	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		__uart_recv_string_callback(USART3);

		USART_ClearITPendingBit(USART3, USART_IT_RXNE);	// 清除中断标志位
	}
}

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
			clear = UART4->SR;		// 清除空闲中断
			clear = UART4->DR;		// 清除空闲中断
			g_rx_str_flag[3] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			#if defined(STM32F10X_HD)
			DMA_Cmd(DMA2_Channel3, DISABLE);
			#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
			DMA_Cmd(DMA1_Stream2, DISABLE);
			#endif
	}

	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		__uart_recv_string_callback(UART4);

		USART_ClearITPendingBit(UART4, USART_IT_RXNE);	// 清除中断标志位
	}
}

/******************************************************************************
* @brief	UART5中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void UART5_IRQHandler(void)
{
	#if defined(STM32F40_41xxx) || defined(STM32F429_439xx)
	volatile uint8_t clear;

	if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)   // 空闲中断
	{
			clear = UART5->SR;		// 清除空闲中断
			clear = UART5->DR;		// 清除空闲中断
			g_rx_str_flag[4] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			DMA_Cmd(DMA1_Stream0, DISABLE);
	}
	#endif
	
	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
		__uart_recv_string_callback(UART5);

		USART_ClearITPendingBit(UART5, USART_IT_RXNE);	// 清除中断标志位
	}
}

/******************************************************************************
* @brief	UART6中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART6_IRQHandler(void)
{
	#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	volatile uint8_t clear;

	if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)   // 空闲中断
	{
			clear = USART6->SR;		// 清除空闲中断
			clear = USART6->DR;		// 清除空闲中断
			g_rx_str_flag[5] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			DMA_Cmd(DMA1_Stream1, DISABLE);
	}
	
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
	{
		__uart_recv_string_callback(USART6);

		USART_ClearITPendingBit(USART6, USART_IT_RXNE);	// 清除中断标志位
	}
	#endif
}
