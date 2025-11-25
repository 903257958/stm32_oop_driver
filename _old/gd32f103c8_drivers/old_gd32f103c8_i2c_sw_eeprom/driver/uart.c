#include "uart.h"

uint8_t uart0_tx_data_buf[UART0_TX_SIZE];		// UART0发送数据缓冲区
uint8_t uart0_rx_data_buf[UART0_RX_SIZE];		// UART0接收数据缓冲区

UARTRXControlBlock_t uart0_rx_control_block;	// UART0接收控制块结构体

/* 函数声明 */
static void uart0_dma_init(void);
static void uart0_control_block_init(void);

/******************************************************************************
 * @brief	初始化UART0
 * @param	baud	:  波特率
 * @return	无
 ******************************************************************************/											
void uart0_init(uint32_t baud)
{
	rcu_periph_clock_enable(RCU_USART0);
	rcu_periph_clock_enable(RCU_GPIOA);
	
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
	
	/* 配置USART */
	usart_deinit(USART0);
	usart_baudrate_set(USART0, baud);
	usart_parity_config(USART0, USART_PM_NONE);
	usart_word_length_set(USART0, USART_WL_8BIT);
	usart_stop_bit_set(USART0, USART_STB_1BIT);
	usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
	usart_receive_config(USART0, USART_RECEIVE_ENABLE);
	usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);

	/* 配置中断 */
	if (USART0 == USART0)		nvic_irq_enable(USART0_IRQn, 0, 0);
	else if (USART0 == USART1)	nvic_irq_enable(USART1_IRQn, 0, 0);
	else if (USART0 == USART2)	nvic_irq_enable(USART2_IRQn, 0, 0);
	usart_interrupt_enable(USART0, USART_INT_IDLE);
	
	/* 配置DMA */
	uart0_dma_init();

	/* UART控制块初始化 */
	uart0_control_block_init();

	/* 开启USART */
	usart_enable(USART0);
}

/******************************************************************************
 * @brief	UART0配置DMA
 * @param	无
 * @return	无
 ******************************************************************************/											
static void uart0_dma_init(void)
{
	/* 配置DMA */
	rcu_periph_clock_enable(RCU_DMA0);

	dma_deinit(DMA0, DMA_CH4);

	dma_parameter_struct dma_init_struct;
	dma_init_struct.periph_addr = USART0 + 4;						// 外设基地址，数据寄存器偏移0x04
	dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;		// 外设数据宽度
	dma_init_struct.memory_addr = (uint32_t)uart0_rx_data_buf;		// 内存基地址
	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;			// 内存数据宽度
	dma_init_struct.number = UART0_RX_MAX + 1;						// 只有空闲中断才能判断接收完成，不会产生DMA完成中断
	dma_init_struct.priority = DMA_PRIORITY_HIGH;					// 优先级
	dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;		// 外设不递增
	dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;		// 内存递增
	dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;			// 从外设到内存
	dma_init(DMA0, DMA_CH4, &dma_init_struct);

	dma_circulation_disable(DMA0, DMA_CH4);
	dma_channel_enable(DMA0, DMA_CH4);
}

/******************************************************************************
 * @brief	UART0控制块初始化
 * @param	无
 * @return	无
 ******************************************************************************/
static void uart0_control_block_init(void)
{
	/* 初始化索引指针指向索引数组的第0位 */
	uart0_rx_control_block.index_in = &uart0_rx_control_block.index_buf[0];
	uart0_rx_control_block.index_out = &uart0_rx_control_block.index_buf[0];

	/* 标记索引数组的end */
	uart0_rx_control_block.index_end = &uart0_rx_control_block.index_buf[INDEX_BUF_NUM - 1];

	/* 标记第一段数据的start为接收数据的第0位 */
	uart0_rx_control_block.index_in->start = &uart0_rx_data_buf[0];
}

/******************************************************************************
 * @brief	UART0发送数据
 * @param	format	:	数据
 * @return	无
 ******************************************************************************/	
void uart0_printf(char *format, ...)
{
	uint16_t i;
	va_list list_data;

	va_start(list_data, format);
	vsprintf((char *)uart0_tx_data_buf, format, list_data);
	va_end(list_data);

	for (i = 0; i < strlen((const char *)uart0_tx_data_buf); i++)
	{
		while (usart_flag_get(USART0, USART_FLAG_TBE) != 1);
		usart_data_transmit(USART0, uart0_tx_data_buf[i]);
	}
	while (usart_flag_get(USART0, USART_FLAG_TBE) != 1);
}

/******************************************************************************
 * @brief	UART0接收数据
 * @param	无
 * @return	无
 ******************************************************************************/
void uart0_recv_data(void)
{
	uint16_t i;
	
	/* 缓冲区中有未处理数据 */
	if (uart0_rx_control_block.index_in != uart0_rx_control_block.index_out)
	{
		/* 打印当前数据段的长度 */
		uart0_printf(	"Received %d bytes this time:\r\n", 
						uart0_rx_control_block.index_out->end - uart0_rx_control_block.index_out->start + 1);

		/* 遍历当前数据段的所有字节并打印 */
		for (i = 0; i < uart0_rx_control_block.index_out->end - uart0_rx_control_block.index_out->start + 1; i++)
		{
			uart0_printf("%c", uart0_rx_control_block.index_out->start[i]);
		}
		uart0_printf("\r\n");

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		uart0_rx_control_block.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (uart0_rx_control_block.index_out == uart0_rx_control_block.index_end)
		{
			uart0_rx_control_block.index_out = &uart0_rx_control_block.index_buf[0];
		}
	}
}

/******************************************************************************
 * @brief	USART0中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART0_IRQHandler(void)
{
	if (usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE) != 0)
	{
		/* 软件先读USART_STAT，再读USART_DATA，可清除空闲中断标志位 */
		usart_flag_get(USART0, USART_FLAG_IDLEF);
		usart_data_receive(USART0);

		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		uart0_rx_control_block.data_cnt += ((UART0_RX_MAX + 1) - dma_transfer_number_get(DMA0, DMA_CH4));

		/* 标记这一段数据的end */
		uart0_rx_control_block.index_in->end = &uart0_rx_data_buf[uart0_rx_control_block.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		uart0_rx_control_block.index_in++;
		if (uart0_rx_control_block.index_in == uart0_rx_control_block.index_end)
		{
			uart0_rx_control_block.index_in = &uart0_rx_control_block.index_buf[0];
		}

		/* 判断数据缓冲区剩余大小 */
		if (UART0_RX_SIZE - uart0_rx_control_block.data_cnt >= UART0_RX_MAX)
		{
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			uart0_rx_control_block.index_in->start = &uart0_rx_data_buf[uart0_rx_control_block.data_cnt];
		}
		else
		{
			/* 剩余大小不够再接收一次数据，回卷 */
			uart0_rx_control_block.index_in->start = &uart0_rx_data_buf[0];
			uart0_rx_control_block.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		dma_channel_disable(DMA0, DMA_CH4);
		dma_transfer_number_config(DMA0, DMA_CH4, UART0_RX_MAX + 1);
		dma_memory_address_config(DMA0, DMA_CH4, (uint32_t)uart0_rx_control_block.index_in->start);
		dma_channel_enable(DMA0, DMA_CH4);
	}
}
