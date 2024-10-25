#include "usart.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define __usart_get_tx_port(USARTx)	(	USARTx == USART1 ? GPIOA : \
										USARTx == USART2 ? GPIOA : \
										USARTx == USART3 ? GPIOB : \
										USARTx == UART4 ? GPIOC : \
										USARTx == UART5 ? GPIOC : \
										(int)0	)
											
#define __usart_get_rx_port(USARTx)	(	USARTx == USART1 ? GPIOA : \
										USARTx == USART2 ? GPIOA : \
										USARTx == USART3 ? GPIOB : \
										USARTx == UART4 ? GPIOC : \
										USARTx == UART5 ? GPIOD : \
										(int)0	)
											
#define __usart_get_tx_pin(USARTx)	(	USARTx == USART1 ? GPIO_Pin_9 : \
										USARTx == USART2 ? GPIO_Pin_2 : \
										USARTx == USART3 ? GPIO_Pin_10 : \
										USARTx == UART4 ? GPIO_Pin_10 : \
										USARTx == UART5 ? GPIO_Pin_12 : \
										(int)0	)
											
#define __usart_get_rx_pin(USARTx)	(	USARTx == USART1 ? GPIO_Pin_10 : \
										USARTx == USART2 ? GPIO_Pin_3 : \
										USARTx == USART3 ? GPIO_Pin_11 : \
										USARTx == UART4 ? GPIO_Pin_11 : \
										USARTx == UART5 ? GPIO_Pin_2 : \
										(int)0	)
											
#define __usart_get_irqn(USARTx)	(	USARTx == USART1 ? USART1_IRQn : \
										USARTx == USART2 ? USART2_IRQn : \
										USARTx == USART3 ? USART3_IRQn : \
										USARTx == UART4 ? UART4_IRQn : \
										USARTx == UART5 ? UART5_IRQn : \
										(int)0	)
											
#define __usart_get_index(USARTx)	(	USARTx == USART1 ? 0 : \
										USARTx == USART2 ? 1 : \
										USARTx == USART3 ? 2 : \
										USARTx == UART4 ? 3 : \
										USARTx == UART5 ? 4 : \
										(int)-1	)
										
#define	__usart_config_clock_enable(USARTx)	{	if (USARTx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
												else if (USARTx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
												else if (USARTx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);} \
												else if (USARTx == UART4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);} \
												else if (USARTx == UART5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);} \
												else						{usart_log("usart clock no enable\r\n");} \
											}

#define	__usart_config_clock_disable(USARTx)	{	if (USARTx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);} \
													else if (USARTx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);} \
													else if (USARTx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);} \
													else if (USARTx == UART4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, DISABLE);} \
													else if (USARTx == UART5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, DISABLE);} \
													else						{usart_log("usart clock no disable\r\n");} \
												}

#define	__usart_config_gpio_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{usart_log("usart gpio clock no enable\r\n");} \
												}

#define	__usart_config_gpio_clock_disable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);} \
													else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);} \
													else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);} \
													else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, DISABLE);} \
													else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, DISABLE);} \
													else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, DISABLE);} \
													else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, DISABLE);} \
													else					{usart_log("usart gpio clock no disable\r\n");} \
												}

#define	__usart_config_dma_clock_enable(USARTx)	{	if (USARTx == USART1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if (USARTx == USART2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if (USARTx == USART3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if (USARTx == UART4)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);} \
													else						{usart_log("usart clock no enable\r\n");} \
												}

#define	__usart_config_dma_clock_disable(USARTx){	if (USARTx == USART1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);} \
													else if (USARTx == USART2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);} \
													else if (USARTx == USART3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);} \
													else if (USARTx == UART4)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, DISABLE);} \
													else						{usart_log("usart clock no disable\r\n");} \
												}

#define	__usart_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__usart_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __usart_gpio_deinit(port)	GPIO_DeInit(port)

#elif defined(STM32F40_41xxx)

#define __usart_get_tx_port(USARTx)	(	USARTx == USART1 ? GPIOA : \
										USARTx == USART2 ? GPIOA : \
										USARTx == USART3 ? GPIOB : \
										USARTx == UART4 ? GPIOC : \
										USARTx == UART5 ? GPIOC : \
										USARTx == USART6 ? GPIOC : \
										(int)0	)
											
#define __usart_get_rx_port(USARTx)	(	USARTx == USART1 ? GPIOA : \
										USARTx == USART2 ? GPIOA : \
										USARTx == USART3 ? GPIOB : \
										USARTx == UART4 ? GPIOC : \
										USARTx == UART5 ? GPIOD : \
										USARTx == USART6 ? GPIOC : \
										(int)0	)
											
#define __usart_get_tx_pin(USARTx)	(	USARTx == USART1 ? GPIO_Pin_9 : \
										USARTx == USART2 ? GPIO_Pin_2 : \
										USARTx == USART3 ? GPIO_Pin_10 : \
										USARTx == UART4 ? GPIO_Pin_10 : \
										USARTx == UART5 ? GPIO_Pin_12 : \
										USARTx == USART6 ? GPIO_Pin_6 : \
										(int)0	)
											
#define __usart_get_rx_pin(USARTx)	(	USARTx == USART1 ? GPIO_Pin_10 : \
										USARTx == USART2 ? GPIO_Pin_3 : \
										USARTx == USART3 ? GPIO_Pin_11 : \
										USARTx == UART4 ? GPIO_Pin_11 : \
										USARTx == UART5 ? GPIO_Pin_2 : \
										USARTx == USART6 ? GPIO_Pin_7 : \
										(int)0	)
											
#define __usart_get_irqn(USARTx)	(	USARTx == USART1 ? USART1_IRQn : \
										USARTx == USART2 ? USART2_IRQn : \
										USARTx == USART3 ? USART3_IRQn : \
										USARTx == UART4 ? UART4_IRQn : \
										USARTx == UART5 ? UART5_IRQn : \
										USARTx == USART6 ? USART6_IRQn : \
										(int)0	)
											
#define __usart_get_index(USARTx)	(	USARTx == USART1 ? 0 : \
										USARTx == USART2 ? 1 : \
										USARTx == USART3 ? 2 : \
										USARTx == UART4 ? 3 : \
										USARTx == UART5 ? 4 : \
										USARTx == USART6 ? 5 : \
										(int)-1	)
										
#define	__usart_config_clock_enable(USARTx)	{	if (USARTx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);} \
												else if (USARTx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);} \
												else if (USARTx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);} \
												else if (USARTx == UART4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);} \
												else if (USARTx == UART5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);} \
												else if (USARTx == USART6)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);} \
												else						{usart_log("usart clock no enable\r\n");} \
											}

#define	__usart_config_clock_disable(USARTx)	{	if (USARTx == USART1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);} \
													else if (USARTx == USART2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);} \
													else if (USARTx == USART3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);} \
													else if (USARTx == UART4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, DISABLE);} \
													else if (USARTx == UART5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, DISABLE);} \
													else if (USARTx == USART6)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, DISABLE);} \
													else						{usart_log("usart clock no disable\r\n");} \
												}

#define	__usart_config_gpio_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{usart_log("usart gpio clock no enable\r\n");} \
												}

#define	__usart_config_gpio_clock_disable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, DISABLE);} \
													else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, DISABLE);} \
													else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, DISABLE);} \
													else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, DISABLE);} \
													else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, DISABLE);} \
													else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, DISABLE);} \
													else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, DISABLE);} \
													else					{usart_log("usart gpio clock no disable\r\n");} \
												}

#define	__usart_config_dma_clock_enable(USARTx)	{	if (USARTx == USART1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
													else if (USARTx == USART2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if (USARTx == USART3)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if (USARTx == UART4)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if (USARTx == UART5)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if (USARTx == USART6)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
													else						{usart_log("usart clock no enable\r\n");} \
												}

#define	__usart_config_dma_clock_disable(USARTx){	if (USARTx == USART1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, DISABLE);} \
													else if (USARTx == USART2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, DISABLE);} \
													else if (USARTx == USART3)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, DISABLE);} \
													else if (USARTx == UART4)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, DISABLE);} \
													else if (USARTx == UART5)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, DISABLE);} \
													else if (USARTx == USART6)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, DISABLE);} \
													else						{usart_log("usart clock no disable\r\n");} \
												}

#define	__usart_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__usart_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __usart_get_dma_stream(usartx)	(	usartx == USART1 ? DMA2_Stream5 : \
											usartx == USART2 ? DMA1_Stream5 : \
											usartx == USART3 ? DMA1_Stream1 : \
											usartx == UART4 ? DMA1_Stream2 : \
											usartx == UART5 ? DMA1_Stream0 : \
											usartx == USART6 ? DMA2_Stream1 : \
											(int)0)

#define	__usart_get_dma_channel(usartx)		(	usartx == USART1 ? DMA_Channel_4 : \
												usartx == USART2 ? DMA_Channel_4 : \
												usartx == USART3 ? DMA_Channel_4 : \
												usartx == UART4 ? DMA_Channel_4 : \
												usartx == UART5 ? DMA_Channel_4 : \
												usartx == USART6 ? DMA_Channel_5 : \
												(int)0)
												
#define	__usart_get_dma_flag(usartx)		(	usartx == USART1 ? DMA_FLAG_TCIF5 : \
												usartx == USART2 ? DMA_FLAG_TCIF5 : \
												usartx == USART3 ? DMA_FLAG_TCIF1 : \
												usartx == UART4 ? DMA_FLAG_TCIF2 : \
												usartx == UART5 ? DMA_FLAG_TCIF0 : \
												usartx == USART6 ? DMA_FLAG_TCIF1 : \
												(int)0)

#define __usart_gpio_deinit(port)	GPIO_DeInit(port)

#endif

#define MAX_USART_NUM			6									//串口外设最大数量
#define MAX_RX_STRING_LENGTH	100									//接收文本数据包最大长度
#define RX_HEX_PACKET_LENGTH	4									//接收HEX数据包长度
											
static uint8_t gRxByte[MAX_USART_NUM];								//接收字节
static uint8_t gRxByteFlag[MAX_USART_NUM];							//接收字节标志位

static char gRxString[MAX_USART_NUM][MAX_RX_STRING_LENGTH];			//接收文本数据包
static uint8_t gRxStringFlag[MAX_USART_NUM];						//接收文本数据包标志位

static uint8_t gRxHexPacket[MAX_USART_NUM][RX_HEX_PACKET_LENGTH];	//接收HEX数据包
static uint8_t gRxHexPacketFlag[MAX_USART_NUM];						//接收HEX数据包标志位

/*USART私有数据结构体*/
typedef struct {
	USART_GPIO_Port txPort;			// 发送端口
	USART_GPIO_Port rxPort;			// 接收端口
	uint32_t txPin;					// 发送引脚
	uint32_t rxPin;					// 接收引脚
	uint32_t irqn;					// 中断号
	uint8_t index;					// 索引
}USARTPrivData_t;

/* 函数声明 */
static int __usart_dma_recv_enable(USARTDev_t *pDev);
static int __usart_send_byte(USARTDev_t *pDev, uint8_t byte);
static int __usart_send_array(USARTDev_t *pDev, uint8_t *array, uint16_t length);
static int __usart_send_string(USARTDev_t *pDev, char *str);
static int __usart_send_number(USARTDev_t *pDev, uint32_t num, uint8_t length);
static int __usart_send_hex_packet(USARTDev_t *pDev, uint8_t *packet, uint8_t length, uint8_t head, uint8_t end);
static int __usart_printf(USARTDev_t *pDev, char *format, ...);
static uint8_t __usart_recv_byte(USARTDev_t *pDev);
static uint8_t __usart_recv_byte_flag(USARTDev_t *pDev);
static char *__usart_recv_string(USARTDev_t *pDev);
static uint8_t __usart_recv_string_flag(USARTDev_t *pDev);
static uint8_t *__usart_recv_hex_packet(USARTDev_t *pDev);
static uint8_t __usart_recv_hex_packet_flag(USARTDev_t *pDev);
static int __usart_deinit(USARTDev_t *pDev);

/******************************************************************************
 * @brief	初始化USART
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int usart_init(USARTDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 保存私有数据 */
	pDev->pPrivData = (USARTPrivData_t *)malloc(sizeof(USARTPrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	USARTPrivData_t *pPrivData = (USARTPrivData_t *)pDev->pPrivData;
	
	pPrivData->txPort = __usart_get_tx_port(pDev->info.usartx);
	pPrivData->rxPort = __usart_get_rx_port(pDev->info.usartx);
	pPrivData->txPin = __usart_get_tx_pin(pDev->info.usartx);
	pPrivData->rxPin = __usart_get_rx_pin(pDev->info.usartx);
	pPrivData->irqn = __usart_get_irqn(pDev->info.usartx);
	pPrivData->index = __usart_get_index(pDev->info.usartx);

	/* 配置时钟与GPIO */
	__usart_config_clock_enable(pDev->info.usartx);
	__usart_config_gpio_clock_enable(pPrivData->txPort);
	__usart_config_gpio_clock_enable(pPrivData->rxPort);
	__usart_config_io_af_pp(pPrivData->txPort, pPrivData->txPin);
	__usart_config_io_af_pp(pPrivData->rxPort, pPrivData->rxPin);
	
	#if defined(STM32F40_41xxx)
	if (pDev->info.usartx == USART1)
	{
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	}
	else if (pDev->info.usartx == USART2)
	{
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	}
	else if (pDev->info.usartx == USART2)
	{
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	}
	else if (pDev->info.usartx == UART4)
	{
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	}
	else if (pDev->info.usartx == UART5)
	{
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	}
	else if (pDev->info.usartx == USART6)
	{
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	}
	#endif
	
	/* 配置USART */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = pDev->info.baud;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(pDev->info.usartx, &USART_InitStructure);
	
	/* 配置中断 */
	USART_ITConfig(pDev->info.usartx, USART_IT_RXNE, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = pPrivData->irqn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 开启USART */
	USART_Cmd(pDev->info.usartx, ENABLE);
	
	/* 函数指针赋值 */
	pDev->send_byte = __usart_send_byte;
	pDev->send_array = __usart_send_array;
	pDev->send_string = __usart_send_string;
	pDev->send_number = __usart_send_number;
	pDev->send_hex_packet = __usart_send_hex_packet;
	pDev->printf = __usart_printf;
	pDev->recv_byte = __usart_recv_byte;
	pDev->recv_byte_flag = __usart_recv_byte_flag;
	pDev->recv_string = __usart_recv_string;
	pDev->recv_string_flag = __usart_recv_string_flag;
	pDev->recv_hex_packet = __usart_recv_hex_packet;
	pDev->recv_hex_packet_flag = __usart_recv_hex_packet_flag;
	pDev->deinit = __usart_deinit;
	
	pDev->DMAFlag = false;
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	初始化USART并配置为DMA接收数据
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int usart_dma_init(USARTDev_t *pDev)
{
	/* USART初始化 */
	usart_init(pDev);
	
	/* 保存私有数据 */
	USARTPrivData_t *pPrivData = (USARTPrivData_t *)pDev->pPrivData;
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	if (pDev->info.usartx == USART1)			{	pPrivData->DMAChannel = DMA1_Channel5;	}
	else if (pDev->info.usartx == USART2)	{	pPrivData->DMAChannel = DMA1_Channel6;	}
	else if (pDev->info.usartx == USART3)	{	pPrivData->DMAChannel = DMA1_Channel3;	}
	else if (pDev->info.usartx == UART4)		{	pPrivData->DMAChannel = DMA2_Channel3;	}
	else	{return -1;}

	/* 配置DMA */
	__usart_config_dma_clock_enable(pDev->info.usartx);
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(pPrivData->DMAChannel);													// 将DMA的通道寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&pDev->info.usartx->DR;		// DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(gRxString[pPrivData->index]);		// DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;									// 数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = sizeof(gRxString[pPrivData->index]);				// DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					// 外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;								// 内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;				// 数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;						// 数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										// 工作在正常模式，一次传输后自动结束
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;								// 中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;										// 没有设置为内存到内存传输
	DMA_Init(pPrivData->DMAChannel, &DMA_InitStructure);

	/* 中断使能 */
	USART_ITConfig(pDev->info.usartx, USART_IT_RXNE, DISABLE);							// 关闭串口接受中断
	USART_ITConfig(pDev->info.usartx, USART_IT_IDLE, ENABLE);							// 使能USART空闲中断

	/* 开启DMA */
	DMA_Cmd(pPrivData->DMAChannel, ENABLE);

	/* 启用USART的DMA请求 */
	USART_DMACmd(pDev->info.usartx, USART_DMAReq_Rx, ENABLE);

	#elif defined(STM32F40_41xxx)
	
	/* 配置DMA */
	__usart_config_dma_clock_enable(pDev->info.usartx);									// 开启DMA时钟
	DMA_DeInit(__usart_get_dma_stream(pDev->info.usartx));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = __usart_get_dma_channel(pDev->info.usartx);			// 选择DMA通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&pDev->info.usartx->DR;		// DMA外设基地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(gRxString[pPrivData->index]);	// DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;								// 从外设读取发送到内存
    DMA_InitStructure.DMA_BufferSize = sizeof(gRxString[pPrivData->index]);				// DMA通道的DMA缓存的大小
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
    DMA_Init(__usart_get_dma_stream(pDev->info.usartx), &DMA_InitStructure);
					
	/* 中断使能 */
	USART_ITConfig(pDev->info.usartx, USART_IT_RXNE, DISABLE);							// 关闭串口接受中断
	USART_ITConfig(pDev->info.usartx, USART_IT_IDLE, ENABLE);							// 使能USART空闲中断

	/* 开启DMA */
    DMA_Cmd(__usart_get_dma_stream(pDev->info.usartx), ENABLE);

	/* 启用USART的DMA请求 */
	USART_DMACmd(pDev->info.usartx, USART_DMAReq_Rx, ENABLE);
	
	#endif
	
	/* 函数指针赋值 */
	pDev->dma_recv_enable = __usart_dma_recv_enable;
	
	pDev->DMAFlag = true;
	
	return 0;
}

/******************************************************************************
 * @brief	USART开启一次DMA接收
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __usart_dma_recv_enable(USARTDev_t *pDev)
{
	USARTPrivData_t *pPrivData = (USARTPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag || !pDev->DMAFlag)
		return -1;
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	/* 将接收的内存部分数据清0 */
	memset(gRxString[pPrivData->index], 0, sizeof(gRxString[pPrivData->index]));
	
	/* 重新设置传输数据长度 */
	DMA_SetCurrDataCounter(pPrivData->DMAChannel, sizeof(gRxString[pPrivData->index]));

	/* 重新打开DMA */
	DMA_Cmd(pPrivData->DMAChannel, ENABLE);
	
	#elif defined(STM32F40_41xxx)
	
	/* 将接收的内存部分数据清0 */
	memset(gRxString[pPrivData->index], 0, sizeof(gRxString[pPrivData->index]));

	/* 清除标志位 */
	DMA_ClearFlag(__usart_get_dma_stream(pDev->info.usartx), __usart_get_dma_flag(pDev->info.usartx));
	USART_ClearFlag(pDev->info.usartx, USART_FLAG_IDLE);

	/* 重新设置传输数据长度 */
	DMA_SetCurrDataCounter(__usart_get_dma_stream(pDev->info.usartx), sizeof(gRxString[pPrivData->index]));

	/* 重新打开DMA */
	DMA_Cmd(__usart_get_dma_stream(pDev->info.usartx), ENABLE);
	
	#endif

	return 0;
}

/******************************************************************************
 * @brief	USART发送一个字节
 * @param	pDev	:  USARTDev_t结构体指针
 * @param	byte	:  要发送的字节
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/	
static int __usart_send_byte(USARTDev_t *pDev, uint8_t byte)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	USART_SendData(pDev->info.usartx, byte);	//写DR寄存器
	while (RESET == USART_GetFlagStatus(pDev->info.usartx, USART_FLAG_TXE));	//等待TXE置1，不需要手动清除标志位
	
	return 0;
}

/******************************************************************************
 * @brief	USART发送一个数组
 * @param	pDev	:  USARTDev_t结构体指针
 * @param	array	:  要发送的数组首地址
 * @param	length	:  要发送的数组长度
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __usart_send_array(USARTDev_t *pDev, uint8_t *array, uint16_t length)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	while (length--)
    {
        __usart_send_byte(pDev, *array++);
    }
	
	return 0;
}

/******************************************************************************
 * @brief	USART发送一个字符串
 * @param	pDev	:  USARTDev_t结构体指针
 * @param	str		:  要发送的字符串首地址
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __usart_send_string(USARTDev_t *pDev, char *str)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	while (*str != '\0')
	{
		__usart_send_byte(pDev, *str++);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	次方函数（USART发送一个数字用到）
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
 * @brief	USART发送一个数字
 * @param	pDev	:  USARTDev_t结构体指针
 * @param	num		:  要发送的数字
 * @param	length	:  要发送的数字长度
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __usart_send_number(USARTDev_t *pDev, uint32_t num, uint8_t length)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i;
	for(i = 0;i < length;i++)
	{
		__usart_send_byte(pDev, num / pow(10, length - i - 1) % 10 + '0');
	}
	
	return 0;
}

/******************************************************************************
 * @brief	USART发送HEX数据包（0xFF为包头，0xFE为包尾）
 * @param	pDev	:  USARTDev_t结构体指针
 * @param	packet	:  要发送的HEX数据包首地址
 * @param	length	:  要发送的HEX数据包长度
 * @param	packet	:  要发送的HEX数据包头
 * @param	length	:  要发送的HEX数据包尾
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __usart_send_hex_packet(USARTDev_t *pDev, uint8_t *packet, uint8_t length, uint8_t head, uint8_t end)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	__usart_send_byte(pDev, head);	//发送包头
	__usart_send_array(pDev, packet, length);	//发送数据包
	__usart_send_byte(pDev, end);	//发送包尾
	
	return 0;
}

/******************************************************************************
 * @brief	printf重定义（仅USART1可用）
 ******************************************************************************/
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, ch);	//写DR寄存器
	while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE));	//等待TXE置1，不需要手动清除标志位
	return ch;
}

/******************************************************************************
 * @brief	printf函数封装（多个串口都可使用）
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __usart_printf(USARTDev_t *pDev, char *format, ...)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	char str[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(str, format, arg);
	va_end(arg);
	__usart_send_string(pDev, str);
	
	return 0;
}

/******************************************************************************
 * @brief	返回USART接收的一个字节
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	USART接收的一个字节
 ******************************************************************************/
static uint8_t __usart_recv_byte(USARTDev_t *pDev)
{
	USARTPrivData_t *pPrivData = pDev->pPrivData;
	
	return gRxByte[pPrivData->index];
}

/******************************************************************************
 * @brief	获取USART接收一个字节标志位（并自动清除）
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	USART接收一个字节标志位
 ******************************************************************************/
static uint8_t __usart_recv_byte_flag(USARTDev_t *pDev)
{
	USARTPrivData_t *pPrivData = pDev->pPrivData;
	
	if (gRxByteFlag[pPrivData->index] == 1)
	{
		gRxByteFlag[pPrivData->index] = 0;
		return 1;
	}
	return 0;
}

/******************************************************************************
 * @brief	返回USART接收的文本数据包的首地址
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	USART接收的文本数据包的首地址
 ******************************************************************************/
static char *__usart_recv_string(USARTDev_t *pDev)
{
	USARTPrivData_t *pPrivData = pDev->pPrivData;
	
	return gRxString[pPrivData->index];
}

/******************************************************************************
 * @brief	返回USART接收的文本数据包标志位（并自动清除）
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	USART接收的文本数据包标志位
 ******************************************************************************/
static uint8_t __usart_recv_string_flag(USARTDev_t *pDev)
{
	USARTPrivData_t *pPrivData = pDev->pPrivData;
	
	if (gRxStringFlag[pPrivData->index] == 1)
	{
		gRxStringFlag[pPrivData->index] = 0;
		
		return 1;
	}
	return 0;
}

/******************************************************************************
 * @brief	返回USART接收的HEX数据包的首地址
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	USART接收的HEX数据包的首地址
 ******************************************************************************/
static uint8_t *__usart_recv_hex_packet(USARTDev_t *pDev)
{
	USARTPrivData_t *pPrivData = pDev->pPrivData;
	
	return gRxHexPacket[pPrivData->index];
}

/******************************************************************************
 * @brief	返回USART接收的HEX数据包标志位（并自动清除）
 * @param	pDev	:  USARTDev_t结构体指针
 * @return	USART接收的HEX数据包标志位
 ******************************************************************************/
static uint8_t __usart_recv_hex_packet_flag(USARTDev_t *pDev)
{
	USARTPrivData_t *pPrivData = pDev->pPrivData;
	
	if (gRxHexPacketFlag[pPrivData->index] == 1)
	{
		gRxHexPacketFlag[pPrivData->index] = 0;
		return 1;
	}
	return 0;
}

/******************************************************************************
 * @brief	去初始化USART
 * @param	pDev   :  USARTDev_t
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __usart_deinit(USARTDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	USARTPrivData_t *pPrivData = (USARTPrivData_t *)pDev->pPrivData;
	
	/*关闭时钟*/
	__usart_config_clock_disable(pDev->info.usartx);
	__usart_config_gpio_clock_disable(pPrivData->txPort);
	__usart_config_gpio_clock_disable(pPrivData->rxPort);
	if (pDev->DMAFlag)
	{
		__usart_config_dma_clock_disable(pDev->info.usartx);
	}
	
	/*复位GPIO*/
	__usart_gpio_deinit(pPrivData->txPort);
	__usart_gpio_deinit(pPrivData->rxPort);
	
	/*释放私有数据内存*/
	free(pDev->pPrivData);
    pDev->pPrivData = NULL;
	
	pDev->DMAFlag = false;	//修改初始化标志
	pDev->initFlag = false;
	
	return 0;
}

/******************************************************************************
 * @brief	USART接收一个字节中断回调函数
 * @param	usartx	：	串口外设
 * @return	无
 ******************************************************************************/
static void __usart_recv_byte_callback(USARTx usartx)
{
	uint8_t index;
	if (usartx == USART1)		{index = 0;}
	else if (usartx == USART2)	{index = 1;}
	else if (usartx == USART3)	{index = 2;}
	else if (usartx == UART4)	{index = 3;}
	else if (usartx == UART5)	{index = 4;}
	else if (usartx == USART6)	{index = 5;}
	
	gRxByte[index] = USART_ReceiveData(usartx);
	gRxByteFlag[index] = 1;
}

/******************************************************************************
 * @brief	USART接收文本数据包中断回调函数
 * @param	usartx	：	串口外设
 * @return	无
 ******************************************************************************/
static void __usart_recv_string_callback(USARTx usartx)
{
	uint8_t index;
	if (usartx == USART1)		{index = 0;}
	else if (usartx == USART2)	{index = 1;}
	else if (usartx == USART3)	{index = 2;}
	else if (usartx == UART4)	{index = 3;}
	else if (usartx == UART5)	{index = 4;}
	else if (usartx == USART6)	{index = 5;}
	
	static uint8_t rx_string_num = 0; 						//接收到文本数据包的数据个数
	
	uint8_t rx_byte = USART_ReceiveData(usartx);
	
	if (rx_string_num < (sizeof(gRxString[index]) - 1)) 	//确保不越界
	{
		gRxString[index][rx_string_num++] = rx_byte;
		if (rx_byte == '\n' && rx_string_num > 1 && gRxString[index][rx_string_num - 2] == '\r')	//检查是否接收到完整的数据包
		{
			gRxString[index][rx_string_num - 2] = '\0';	//结束符
			rx_string_num = 0; 							//重置接收计数器
			gRxStringFlag[index] = 1; 					//标记数据包已接收完成
		}
	}
	else
	{
		rx_string_num = 0;								//如果接收到的数据超出了数组的大小，重置接收计数器
		gRxStringFlag[index] = 0;
	}
}

/******************************************************************************
 * @brief	USART接收HEX数据包中断回调函数
 * @param	usartx	：	串口外设
 * @param	head	：	要接收的HEX数据包头
 * @param	end		：	要接收的HEX数据包尾
 * @return	无
 ******************************************************************************/
static void __usart_recv_hex_packet_callback(USARTx usartx, uint8_t head, uint8_t end)
{
	uint8_t index;
	if (usartx == USART1)		{index = 0;}
	else if (usartx == USART2)	{index = 1;}
	else if (usartx == USART3)	{index = 2;}
	else if (usartx == UART4)	{index = 3;}
	else if (usartx == UART5)	{index = 4;}
	else if (usartx == USART6)	{index = 5;}
	static uint8_t rx_state = 0;			//状态变量
	static uint8_t rx_hex_packet_num = 0;	//接收到HEX数据包的数据个数
	
	uint8_t rx_byte = USART_ReceiveData(usartx);
	
	if (rx_state == 0)
	{
		if (rx_byte == head)	//收到包头
		{
			rx_state = 1;
			rx_hex_packet_num = 0;
		}
	}
	else if (rx_state == 1)
	{
		gRxHexPacket[index][rx_hex_packet_num++] = rx_byte;
		if (rx_hex_packet_num >= RX_HEX_PACKET_LENGTH)
		{
			rx_state = 2;
		}
	}
	else if (rx_state == 2)
	{
		if (rx_byte == end)	//收到包尾
		{
			rx_state = 0;
			gRxHexPacketFlag[index] = 1;
		}
	}
}

/******************************************************************************
 * @brief	USART1中断函数
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
			gRxStringFlag[0] = 1;		// 置接收标志位
		
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
			DMA_Cmd(DMA1_Channel5, DISABLE);
			#elif defined(STM32F40_41xxx)
			DMA_Cmd(DMA2_Stream5, DISABLE);
			#endif
	}
	
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		__usart_recv_byte_callback(USART1);
		__usart_recv_string_callback(USART1);
		__usart_recv_hex_packet_callback(USART1, 0x24, 0xFF);

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);	// 清除中断标志位
	}
}

/******************************************************************************
 * @brief	USART2中断函数
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
			gRxStringFlag[1] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
			DMA_Cmd(DMA1_Channel6, DISABLE);
			#elif defined(STM32F40_41xxx)
			DMA_Cmd(DMA1_Stream5, DISABLE);
			#endif
	}

	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		__usart_recv_byte_callback(USART2);
		__usart_recv_string_callback(USART2);
		__usart_recv_hex_packet_callback(USART2, 0x24, 0xFF);

		USART_ClearITPendingBit(USART2, USART_IT_RXNE);	// 清除中断标志位
	}
}

/******************************************************************************
 * @brief	USART3中断函数
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
			gRxStringFlag[2] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
			DMA_Cmd(DMA1_Channel3, DISABLE);
			#elif defined(STM32F40_41xxx)
			DMA_Cmd(DMA1_Stream1, DISABLE);
			#endif
	}

	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		__usart_recv_byte_callback(USART3);
		__usart_recv_string_callback(USART3);
		__usart_recv_hex_packet_callback(USART3, 0x24, 0xFF);

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
			gRxStringFlag[3] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
			DMA_Cmd(DMA2_Channel3, DISABLE);
			#elif defined(STM32F40_41xxx)
			DMA_Cmd(DMA1_Stream2, DISABLE);
			#endif
	}

	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		__usart_recv_byte_callback(UART4);
		__usart_recv_string_callback(UART4);
		__usart_recv_hex_packet_callback(UART4, 0x24, 0xFF);

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
	#if defined(STM32F40_41xxx)
	volatile uint8_t clear;

	if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)   // 空闲中断
	{
			clear = UART5->SR;		// 清除空闲中断
			clear = UART5->DR;		// 清除空闲中断
			gRxStringFlag[4] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			DMA_Cmd(DMA1_Stream0, DISABLE);
	}
	#endif
	
	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
		__usart_recv_byte_callback(UART5);
		__usart_recv_string_callback(UART5);
		__usart_recv_hex_packet_callback(UART5, 0x24, 0xFF);

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
	#if defined(STM32F40_41xxx)
	volatile uint8_t clear;

	if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)   // 空闲中断
	{
			clear = USART6->SR;		// 清除空闲中断
			clear = USART6->DR;		// 清除空闲中断
			gRxStringFlag[5] = 1;	// 置接收标志位
			
			/* 空闲中断产生，关闭DMA，等待数据处理，在调用 dma_recv_enable 函数后DMA才被重新开启 */
			DMA_Cmd(DMA1_Stream1, DISABLE);
	}
	
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
	{
		__usart_recv_byte_callback(USART6);
		__usart_recv_string_callback(USART6);
		__usart_recv_hex_packet_callback(USART6, 0x24, 0xFF);

		USART_ClearITPendingBit(USART6, USART_IT_RXNE);	// 清除中断标志位
	}
	#endif
}
