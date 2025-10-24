#include "uart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#ifdef USE_STDPERIPH_DRIVER

/**************************** GD32F1 系列 ****************************/
#if defined(GD32F10X_MD) || defined(GD32F10X_HD)

#define uart_io_clock_enable(port)									 \
    do {                                           				     \
        if (port == GPIOA)       rcu_periph_clock_enable(RCU_GPIOA); \
        else if (port == GPIOB)  rcu_periph_clock_enable(RCU_GPIOB); \
        else if (port == GPIOC)  rcu_periph_clock_enable(RCU_GPIOC); \
        else if (port == GPIOD)  rcu_periph_clock_enable(RCU_GPIOD); \
        else if (port == GPIOE)  rcu_periph_clock_enable(RCU_GPIOE); \
        else if (port == GPIOF)  rcu_periph_clock_enable(RCU_GPIOF); \
        else if (port == GPIOG)  rcu_periph_clock_enable(RCU_GPIOG); \
    } while (0)

#define uart_config_io_af_pp(port, pin)	\
        gpio_init(port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, pin)

#define uart_config_io_in_pn(port, pin) \
    	gpio_init(port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, pin)

	/**************************** GD32F10X_MD 系列：UART0、1、2 可用 ****************************/
	#if defined(GD32F10X_MD)

	#define UART0_AVAILABLE
	#define UART1_AVAILABLE
	#define UART2_AVAILABLE

	#define uart_clock_enable(uartx)                                       \
		do {                                                               \
			if (uartx == USART0) 	  rcu_periph_clock_enable(RCU_USART0); \
			else if (uartx == USART1) rcu_periph_clock_enable(RCU_USART1); \
			else if (uartx == USART2) rcu_periph_clock_enable(RCU_USART2); \
		} while (0)

	#define uart_dma_clock_enable(uartx) \
			rcu_periph_clock_enable(RCU_DMA0)

	#define uart_get_irqn(uartx) ((uartx) == USART0 ? USART0_IRQn :	\
								 ((uartx) == USART1 ? USART1_IRQn :	\
								 ((uartx) == USART2 ? USART2_IRQn : (IRQn_Type)-1)))

	#define uart_get_dma_periph(uartx) DMA0

	#define uart_get_dma_channel(uartx) ((uartx) == USART0 ? DMA_CH4 : \
                                    	((uartx) == USART1 ? DMA_CH5 : \
                                    	((uartx) == USART2 ? DMA_CH2 : (dma_channel_enum)-1)))

	#define uart_get_index(uartx) ((uartx) == USART0 ? 0 : \
								  ((uartx) == USART1 ? 1 : \
								  ((uartx) == USART2 ? 2 : (int)-1)))
    
	#endif	/* GD32F1 子系列 */

#endif	/* GD32F1 系列 */

#endif	/* USE_STDPERIPH_DRIVER */

/* 最大串口数量 */
#define MAX_UART_NUM 3

/* 已注册串口设备指针数组 */
static uart_dev_t *uart_registered[MAX_UART_NUM] = {NULL};

/* 函数声明 */
static void uart_dma_init(uart_dev_t *dev);
static void uart_printf(uart_dev_t *dev, const char *format, va_list args);
static void uart_send(uart_dev_t *dev, uint8_t *data, uint32_t len);
static char *uart_recv(uart_dev_t *dev);
static int uart_drv_deinit(uart_dev_t *dev);

#ifdef UART0_AVAILABLE
static void uart0_printf(char *format, ...);
static void uart0_send(uint8_t *data, uint32_t len);
static char *uart0_recv(void);
#endif

#ifdef UART1_AVAILABLE
static void uart1_printf(char *format, ...);
static void uart1_send(uint8_t *data, uint32_t len);
static char *uart1_recv(void);
#endif

#ifdef UART2_AVAILABLE
static void uart2_printf(char *format, ...);
static void uart2_send(uint8_t *data, uint32_t len);
static char *uart2_recv(void);
#endif

/**
 * @brief   初始化 UART ，只能配置支持 DMA 的串口
 * @param[in,out] dev uart_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */												
int uart_drv_init(uart_dev_t *dev)
{
	if (!dev)
		return -1;

	int8_t index;
	
	/* 注册设备 */
	index = uart_get_index(dev->config.uartx);
	if (index >= 0)
		uart_registered[index] = dev;
	
	/* 初始化接收数据控制块 */
	dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];
	dev->rx_cb.index_out = &dev->rx_cb.index_buf[0];
	dev->rx_cb.index_end = &dev->rx_cb.index_buf[INDEX_BUF_NUM - 1];
	dev->rx_cb.index_in->start = dev->config.rx_buf;
	dev->rx_cb.data_cnt = 0;

	uart_clock_enable(dev->config.uartx);
	uart_io_clock_enable(dev->config.tx_port);
	uart_io_clock_enable(dev->config.rx_port);

	uart_config_io_af_pp(dev->config.tx_port, dev->config.tx_pin);	// 串口发送配置为复用推挽输出
	uart_config_io_in_pn(dev->config.rx_port, dev->config.rx_pin);	// 串口接收配置为浮空输入
	
	/* 配置 USART */
	usart_deinit(dev->config.uartx);
	usart_baudrate_set(dev->config.uartx, dev->config.baud);
	usart_word_length_set(dev->config.uartx, USART_WL_8BIT);
	usart_parity_config(dev->config.uartx, USART_PM_NONE);
	usart_stop_bit_set(dev->config.uartx, USART_STB_1BIT);
	usart_transmit_config(dev->config.uartx, USART_TRANSMIT_ENABLE);
	usart_receive_config(dev->config.uartx, USART_RECEIVE_ENABLE);

	/* 配置中断 */
	nvic_irq_enable(uart_get_irqn(dev->config.uartx), 0, 0);
	usart_interrupt_enable(dev->config.uartx, USART_INT_IDLE);
	
	/* 配置 DMA */
	uart_dma_init(dev);

	/* 使能 USART */
	usart_enable(dev->config.uartx);

	#ifdef UART0_AVAILABLE
	if (dev->config.uartx == USART0) {
		dev->printf = uart0_printf;
		dev->send = uart0_send;
		dev->recv = uart0_recv;
	}
	#endif

	#ifdef UART1_AVAILABLE
	if (dev->config.uartx == USART1) {
		dev->printf = uart1_printf;
		dev->send = uart1_send;
		dev->recv = uart1_recv;
	}
	#endif

	#ifdef UART2_AVAILABLE
	if (dev->config.uartx == USART2) {
		dev->printf = uart2_printf;
		dev->send = uart2_send;
		dev->recv = uart2_recv;
	}
	#endif

	dev->deinit = uart_drv_deinit;

	dev->init_flag = true;
	return 0;
}

/**
 * @brief   UART 配置为 DMA 接收数据
 * @param[in] dev uart_dev_t 结构体指针
 */												
static void uart_dma_init(uart_dev_t *dev)
{
	uart_dma_clock_enable(dev->config.uartx);
	dma_deinit(uart_get_dma_periph(dev->config.uartx), uart_get_dma_channel(dev->config.uartx));

	dma_parameter_struct dma_init_struct;
	dma_init_struct.periph_addr = dev->config.uartx + 4;		// 外设基地址，数据寄存器偏移0x04
	dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;	// 外设数据宽度
	dma_init_struct.memory_addr = (uint32_t)dev->config.rx_buf;	// 内存基地址
	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;		// 内存数据宽度
	dma_init_struct.number = dev->config.rx_single_max + 1;		// 只有空闲中断才能判断接收完成，不会产生DMA完成中断
	dma_init_struct.priority = DMA_PRIORITY_HIGH;				// 优先级
	dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;	// 外设不递增
	dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;	// 内存递增
	dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;		// 从外设到内存
	dma_init(uart_get_dma_periph(dev->config.uartx), uart_get_dma_channel(dev->config.uartx), &dma_init_struct);

	dma_circulation_disable(uart_get_dma_periph(dev->config.uartx), uart_get_dma_channel(dev->config.uartx));
	dma_channel_enable(uart_get_dma_periph(dev->config.uartx), uart_get_dma_channel(dev->config.uartx));
	usart_dma_receive_config(dev->config.uartx, USART_RECEIVE_DMA_ENABLE);
}

/**
 * @brief   UART 通用打印函数，内部使用
 * @param[in] dev    uart_dev_t 结构体指针
 * @param[in] format 格式化参数
 * @param[in] args   va_list
 */	
static void uart_printf(uart_dev_t *dev, const char *format, va_list args)
{
	uint16_t i;

	vsnprintf((char *)dev->config.tx_buf, dev->config.tx_buf_size, format, args);
	for (i = 0; i < strlen((const char *)dev->config.tx_buf); i++) {
		while (usart_flag_get(dev->config.uartx, USART_FLAG_TBE) != 1);
		usart_data_transmit(dev->config.uartx, dev->config.tx_buf[i]);
	}
	while (usart_flag_get(dev->config.uartx, USART_FLAG_TC) != 1);
}

/**
 * @brief   UART 通用发送数据函数，内部使用
 * @param[in] dev  uart_dev_t 结构体指针
 * @param[in] data 数据
 * @param[in] len  数据长度
 */	
static void uart_send(uart_dev_t *dev, uint8_t *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++) {
		usart_data_transmit(dev->config.uartx, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (usart_flag_get(dev->config.uartx, USART_FLAG_TBE) == RESET);	// 等待发送完成
	}
}

/**
 * @brief   UART 通用接收函数，内部使用
 * @param[in] dev uart_dev_t 结构体指针
 * @return	接收到数据字符串的首地址，未接收到数据则为 NULL
 */	
static char *uart_recv(uart_dev_t *dev)
{
	char *ret_str;

	/* 缓冲区中有未处理数据 */
	if (dev->rx_cb.index_in != dev->rx_cb.index_out) {
		/* 末尾加上'\0'，确保可以作为字符串处理 */
        *(dev->rx_cb.index_out->end + 1) = '\0';

        ret_str = (char *)dev->rx_cb.index_out->start;

		/* 处理完当前数据段后，移动index_out指针到下一个位置，准备处理下一段数据 */
		dev->rx_cb.index_out++;

		/* 如果index_out到达索引数组末尾，则回卷到开始位置 */
		if (dev->rx_cb.index_out == dev->rx_cb.index_end)
			dev->rx_cb.index_out = &dev->rx_cb.index_buf[0];

		return ret_str;
	}

	return NULL;
}

/**
 * @brief   去初始化 UART
 * @param[in,out] dev uart_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
static int uart_drv_deinit(uart_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	dev->init_flag = false;

	return 0;
}

#ifdef UART2_AVAILABLE
/**
 * @brief   UART0 打印
 * @param[in] format 格式化参数
 * @param[in] args   va_list
 */	
static void uart0_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART0)];
	va_list args;

	va_start(args, format);
	uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/**
 * @brief   UART0 发送数据
 * @param[in] data 数据
 * @param[in] len  数据长度
 */	
static void uart0_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART0)];
	uart_send(dev, data, len);
}

/**
 * @brief   UART0 接收数据
 * @return	接收到数据字符串的首地址，未接收到数据则为 NULL
 */
static char *uart0_recv(void)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART0)];
	return uart_recv(dev);
}
#endif

#ifdef UART1_AVAILABLE
/**
 * @brief   UART1 打印
 * @param[in] format 格式化参数
 * @param[in] args   va_list
 */	
static void uart1_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART1)];
	va_list args;

	va_start(args, format);
	uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/**
 * @brief   UART1 发送数据
 * @param[in] data 数据
 * @param[in] len  数据长度
 */	
static void uart1_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART1)];
	uart_send(dev, data, len);
}

/**
 * @brief   UART1 接收数据
 * @return	接收到数据字符串的首地址，未接收到数据则为 NULL
 */
static char *uart1_recv(void)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART1)];
	return uart_recv(dev);
}
#endif

#ifdef UART2_AVAILABLE
/**
 * @brief   UART2 打印
 * @param[in] format 格式化参数
 * @param[in] args   va_list
 */	
static void uart2_printf(char *format, ...)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART2)];
	va_list args;

	va_start(args, format);
	uart_printf(dev, format, args);  // 转发参数
	va_end(args);
}

/**
 * @brief   UART2 发送数据
 * @param[in] data 数据
 * @param[in] len  数据长度
 */	
static void uart2_send(uint8_t *data, uint32_t len)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART2)];
	uart_send(dev, data, len);
}

/**
 * @brief   UART2 接收数据
 * @return	接收到数据字符串的首地址，未接收到数据则为 NULL
 */
static char *uart2_recv(void)
{
	uart_dev_t *dev = uart_registered[uart_get_index(USART2)];
	return uart_recv(dev);
}
#endif


#ifdef UART0_AVAILABLE
/**
 * @brief   UART0 中断函数
 */	
void USART0_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registered[uart_get_index(USART0)];	// 根据索引找到已注册的串口设备
	if (!dev)
		return;
	
	if (usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE) != RESET) {
		/* 软件先读 USART_STAT ，再读 USART_DATA ，可清除空闲中断标志位 */
		usart_flag_get(USART0, USART_FLAG_IDLEF);
		usart_data_receive(USART0);

		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - dma_transfer_number_get(DMA0, DMA_CH4));

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max) {
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		} else {
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		dma_channel_disable(DMA0, DMA_CH4);
		dma_transfer_number_config(DMA0, DMA_CH4, dev->config.rx_single_max + 1);
		dma_memory_address_config(DMA0, DMA_CH4, (uint32_t)dev->rx_cb.index_in->start);
		dma_channel_enable(DMA0, DMA_CH4);
	}
}
#endif

#ifdef UART1_AVAILABLE
/**
 * @brief   UART1 中断函数
 */	
void USART1_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registered[uart_get_index(USART1)];	// 根据索引找到已注册的串口设备
	if (!dev)
		return;
	
	if (usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE) != RESET) {
		/* 软件先读 USART_STAT ，再读 USART_DATA ，可清除空闲中断标志位 */
		usart_flag_get(USART1, USART_FLAG_IDLEF);
		usart_data_receive(USART1);

		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - dma_transfer_number_get(DMA0, DMA_CH5));

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max) {
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		} else {
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		dma_channel_disable(DMA0, DMA_CH5);
		dma_transfer_number_config(DMA0, DMA_CH5, dev->config.rx_single_max + 1);
		dma_memory_address_config(DMA0, DMA_CH5, (uint32_t)dev->rx_cb.index_in->start);
		dma_channel_enable(DMA0, DMA_CH5);
	}
}
#endif

#ifdef UART2_AVAILABLE
/**
 * @brief   UART2 中断函数
 */	
void USART2_IRQHandler(void)
{
	volatile uint8_t clear;
	uart_dev_t *dev = uart_registered[uart_get_index(USART2)];	// 根据索引找到已注册的串口设备
	if (!dev)
		return;
	
	if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE) != RESET) {
		/* 软件先读 USART_STAT ，再读 USART_DATA ，可清除空闲中断标志位 */
		usart_flag_get(USART2, USART_FLAG_IDLEF);
		usart_data_receive(USART2);

		/* DMA传输数据总量 - DMA数据剩余量 = 已接收数据量 */
		dev->rx_cb.data_cnt += ((dev->config.rx_single_max + 1) - dma_transfer_number_get(DMA0, DMA_CH2));

		/* 标记这一段数据的end */
		dev->rx_cb.index_in->end = &dev->config.rx_buf[dev->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		dev->rx_cb.index_in++;
		if (dev->rx_cb.index_in == dev->rx_cb.index_end)
			dev->rx_cb.index_in = &dev->rx_cb.index_buf[0];

		/* 判断数据缓冲区剩余大小 */
		if (dev->config.rx_buf_size - dev->rx_cb.data_cnt >= dev->config.rx_single_max) {
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[dev->rx_cb.data_cnt];
		} else {
			/* 剩余大小不够再接收一次数据，回卷 */
			dev->rx_cb.index_in->start = &dev->config.rx_buf[0];
			dev->rx_cb.data_cnt = 0;
		}

		/* DMA准备下一次接收 */
		dma_channel_disable(DMA0, DMA_CH2);
		dma_transfer_number_config(DMA0, DMA_CH2, dev->config.rx_single_max + 1);
		dma_memory_address_config(DMA0, DMA_CH2, (uint32_t)dev->rx_cb.index_in->start);
		dma_channel_enable(DMA0, DMA_CH2);
	}
}
#endif
