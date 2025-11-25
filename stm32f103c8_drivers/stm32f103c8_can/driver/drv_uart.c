#include "drv_uart.h"
#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/* 硬件信息结构体 */
typedef struct {
	uart_periph_t uart_periph;
	iqrn_type_t iqrn;
	dma_channel_t dma_channel;
#if DRV_UART_PLATFORM_STM32F4
	dma_stream_t dma_stream;
	uint32_t dma_tcif_flag;
	uint8_t af;
#endif
	uint8_t idx;
} uart_hw_info_t;

/* 串口硬件信息列表 */
static const uart_hw_info_t uart_hw_info_table[] = {
#if DRV_UART_PLATFORM_STM32F1
	{ USART1, USART1_IRQn, DMA1_Channel5, 0 },
	{ USART2, USART2_IRQn, DMA1_Channel6, 1 },
	{ USART3, USART3_IRQn, DMA1_Channel3, 2 },
#if defined(STM32F10X_HD)
	{ UART4,  UART4_IRQn,  DMA2_Channel3, 3 },
#endif
#endif	/* DRV_UART_PLATFORM_STM32F1 */

#if DRV_UART_PLATFORM_STM32F4
#if defined(STM32F40_41xxx) || defined(STM32F429_439xx)
	{ USART1, USART1_IRQn, DMA_Channel_4, DMA2_Stream5, DMA_FLAG_TCIF5, GPIO_AF_USART1, 0 },
	{ USART2, USART2_IRQn, DMA_Channel_4, DMA1_Stream5, DMA_FLAG_TCIF5, GPIO_AF_USART2, 1 },
	{ USART3, USART3_IRQn, DMA_Channel_4, DMA1_Stream1, DMA_FLAG_TCIF1, GPIO_AF_USART3, 2 },
	{ UART4,  UART4_IRQn,  DMA_Channel_4, DMA1_Stream2, DMA_FLAG_TCIF2, GPIO_AF_UART4,  3 },
	{ UART5,  UART5_IRQn,  DMA_Channel_4, DMA1_Stream0, DMA_FLAG_TCIF0, GPIO_AF_UART5,  4 },
	{ USART6, USART6_IRQn, DMA_Channel_5, DMA2_Stream1, DMA_FLAG_TCIF1, GPIO_AF_USART6, 5 },
#elif defined(STM32F411xE)
	{ USART1, USART1_IRQn, DMA_Channel_4, DMA2_Stream5, DMA_FLAG_TCIF5, GPIO_AF_USART1, 0 },
	{ USART2, USART2_IRQn, DMA_Channel_4, DMA1_Stream5, DMA_FLAG_TCIF5, GPIO_AF_USART2, 1 },
	{ USART6, USART6_IRQn, DMA_Channel_5, DMA2_Stream1, DMA_FLAG_TCIF1, GPIO_AF_USART6, 2 },
#endif
#endif	/* DRV_UART_PLATFORM_STM32F4 */

#if DRV_UART_PLATFORM_GD32F1
	{ USART0, USART0_IRQn, DMA_CH4, 0 },
	{ USART1, USART1_IRQn, DMA_CH5, 1 },
	{ USART2, USART2_IRQn, DMA_CH2, 2 },
#endif	/* DRV_UART_PLATFORM_GD32F1 */
};

#define MAX_UART_NUM	(sizeof(uart_hw_info_table) / sizeof(uart_hw_info_t))

/**
 * @brief	获取串口硬件信息
 * @param[in] uart_periph 串口外设
 * @return	成功返回对应硬件信息指针，失败返回 NULL
 */
static inline const uart_hw_info_t* uart_get_hw_info(uart_periph_t uart_periph)
{
    for (uint8_t i = 0; i < MAX_UART_NUM; i++) {
        if (uart_hw_info_table[i].uart_periph == uart_periph) {
            return &uart_hw_info_table[i];
        }
    }
    return NULL;
}

/**
 * @brief	使能串口外设时钟
 * @param[in] uart_periph 串口外设
 */
static void uart_hw_uart_clock_enable(uart_periph_t uart_periph)
{
#if DRV_UART_PLATFORM_STM32F1
    switch ((uint32_t)uart_periph) {
    case (uint32_t)USART1: RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); break;
    case (uint32_t)USART2: RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); break;
    case (uint32_t)USART3: RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); break;
    case (uint32_t)UART4:  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);  break;
	default: break;
    }
#elif DRV_UART_PLATFORM_STM32F4
    switch ((uint32_t)uart_periph) {
    case (uint32_t)USART1: RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); break;
    case (uint32_t)USART2: RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); break;
    case (uint32_t)USART3: RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); break;
    case (uint32_t)UART4:  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);  break;
    case (uint32_t)UART5:  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);  break;
    case (uint32_t)USART6: RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); break;
	default: break;
    }
#elif DRV_UART_PLATFORM_GD32F1
	switch ((uint32_t)uart_periph) {
    case (uint32_t)USART0: rcu_periph_clock_enable(RCU_USART0); break;
    case (uint32_t)USART1: rcu_periph_clock_enable(RCU_USART1); break;
    case (uint32_t)USART2: rcu_periph_clock_enable(RCU_USART2); break;
	default: break;
    }
#endif
}

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void uart_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_UART_PLATFORM_STM32F1
    switch ((uint32_t)port) {
    case (uint32_t)GPIOA: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); break;
    case (uint32_t)GPIOB: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); break;
    case (uint32_t)GPIOC: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); break;
    case (uint32_t)GPIOD: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); break;
    case (uint32_t)GPIOE: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); break;
    case (uint32_t)GPIOF: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); break;
    case (uint32_t)GPIOG: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_UART_PLATFORM_STM32F4
    switch ((uint32_t)port) {
    case (uint32_t)GPIOA: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); break;
    case (uint32_t)GPIOB: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); break;
    case (uint32_t)GPIOC: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); break;
    case (uint32_t)GPIOD: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); break;
    case (uint32_t)GPIOE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); break;
    case (uint32_t)GPIOF: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); break;
    case (uint32_t)GPIOG: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_UART_PLATFORM_GD32F1
	switch ((uint32_t)port) {
    case (uint32_t)GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
    case (uint32_t)GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
    case (uint32_t)GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
    case (uint32_t)GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
    case (uint32_t)GPIOE: rcu_periph_clock_enable(RCU_GPIOE); break;
    case (uint32_t)GPIOF: rcu_periph_clock_enable(RCU_GPIOF); break;
    case (uint32_t)GPIOG: rcu_periph_clock_enable(RCU_GPIOG); break;
	default: break;
    }
#endif
}

/**
 * @brief	使能 DMA 时钟
 * @param[in] uart_periph 串口外设
 */
static void uart_hw_dma_clock_enable(uart_periph_t uart_periph)
{
#if DRV_UART_PLATFORM_STM32F1
    if (uart_periph == USART1 || uart_periph == USART2 || uart_periph == USART3)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    else if (uart_periph == UART4)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

#elif DRV_UART_PLATFORM_STM32F4
    if (uart_periph == USART1 || uart_periph == USART6)
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    else
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

#elif DRV_UART_PLATFORM_GD32F1
	rcu_periph_clock_enable(RCU_DMA0);
#endif
}

#if DRV_UART_PLATFORM_STM32F4
/**
 * @brief	获取 GPIO 引脚源（STM32F4特有）
 * @param[in] pin GPIO 引脚
 * @return	引脚源编号（如9对应PinSource9）
 */
static inline uint8_t uart_hw_get_gpio_pin_source(gpio_pin_t pin)
{
    for (uint8_t i = 0; i < 16; i++)
        if (pin & (1 << i))
			return i;  // GPIO_Pin = 1<<i，对应 PinSource = i
    return 0xFF;
}
#endif	/* DRV_UART_PLATFORM_STM32F4 */

/**
 * @brief	初始化 GPIO 为串口功能
 * @param[in] cfg uart_cfg_t 结构体指针
 */
static void uart_hw_gpio_init(const uart_cfg_t *cfg)
{
#if DRV_UART_PLATFORM_STM32F1
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = cfg->tx_pin;
    GPIO_Init(cfg->tx_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = cfg->rx_pin;
    GPIO_Init(cfg->rx_port, &GPIO_InitStructure);

#elif DRV_UART_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = cfg->tx_pin;
    GPIO_Init(cfg->tx_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->rx_pin;
    GPIO_Init(cfg->rx_port, &GPIO_InitStructure);

	const uart_hw_info_t *hw_info = uart_get_hw_info(cfg->uart_periph);
	GPIO_PinAFConfig(cfg->tx_port, uart_hw_get_gpio_pin_source(cfg->tx_pin), hw_info->af);
	GPIO_PinAFConfig(cfg->rx_port, uart_hw_get_gpio_pin_source(cfg->rx_pin), hw_info->af);

#elif DRV_UART_PLATFORM_GD32F1
	gpio_init(cfg->tx_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, cfg->tx_pin);
	gpio_init(cfg->rx_port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, cfg->rx_pin);
#endif
}

/**
 * @brief	初始化串口外设
 * @param[in] cfg uart_cfg_t 结构体指针
 */
static void uart_hw_uart_init(const uart_cfg_t *cfg)
{
	const uart_hw_info_t *hw_info = uart_get_hw_info(cfg->uart_periph);

#if DRV_UART_PLATFORM_STM32F1 || DRV_UART_PLATFORM_STM32F4
	/* 配置 USART */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = cfg->baud;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(cfg->uart_periph, &USART_InitStructure);

	/* 配置中断 */
	USART_ITConfig(cfg->uart_periph, USART_IT_IDLE, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = hw_info->iqrn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = cfg->rx_pre_priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = cfg->rx_sub_priority;
	NVIC_Init(&NVIC_InitStructure);

	/* 使能 USART */
	USART_Cmd(cfg->uart_periph, ENABLE);

#elif DRV_UART_PLATFORM_GD32F1
	/* 配置 USART */
	usart_deinit(cfg->uart_periph);
	usart_baudrate_set(cfg->uart_periph, cfg->baud);
	usart_word_length_set(cfg->uart_periph, USART_WL_8BIT);
	usart_parity_config(cfg->uart_periph, USART_PM_NONE);
	usart_stop_bit_set(cfg->uart_periph, USART_STB_1BIT);
	usart_transmit_config(cfg->uart_periph, USART_TRANSMIT_ENABLE);
	usart_receive_config(cfg->uart_periph, USART_RECEIVE_ENABLE);

	/* 配置中断 */
	nvic_irq_enable(hw_info->iqrn, cfg->rx_pre_priority, cfg->rx_sub_priority);
	usart_interrupt_enable(cfg->uart_periph, USART_INT_IDLE);
	
	/* 使能 USART */
	usart_enable(cfg->uart_periph);
#endif
}

/**
 * @brief	初始化 DMA 用于串口接收
 * @param[in] cfg uart_cfg_t 结构体指针
 */
static void uart_hw_dma_init(const uart_cfg_t *cfg)
{
	const uart_hw_info_t *hw_info = uart_get_hw_info(cfg->uart_periph);
	
#if DRV_UART_PLATFORM_STM32F1
	DMA_DeInit(hw_info->dma_channel);
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&cfg->uart_periph->DR;	// 外设基地址
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		// 外设数据宽度为8位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// 外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)cfg->rx_buf;				// DMA内存基地址
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				// 内存数据宽度为8位
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						// 内存地址寄存器递增
	DMA_InitStructure.DMA_BufferSize = RX_SINGLE_MAX + 1;						// 指定DMA缓冲区大小为单次最大接收量加1，只有空闲中断才能判断接收完成
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;							// 数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								// 工作在正常模式，一次传输后自动结束
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								// 没有设置为内存到内存传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							// 高优先级
	DMA_Init(hw_info->dma_channel, &DMA_InitStructure);
	DMA_Cmd(hw_info->dma_channel, ENABLE);
	USART_DMACmd(cfg->uart_periph, USART_DMAReq_Rx, ENABLE);

#elif DRV_UART_PLATFORM_STM32F4
	DMA_DeInit(hw_info->dma_stream);
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = hw_info->dma_channel;						// DMA通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&cfg->uart_periph->DR;	// DMA外设基地址
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		// 外设数据宽度为8位
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// 外设地址寄存器不变
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)cfg->rx_buf;				// DMA内存基地址
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				// 内存数据宽度为8位
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						// 内存地址寄存器递增
	DMA_InitStructure.DMA_BufferSize = RX_SINGLE_MAX + 1;						// 指定DMA缓冲区大小为单次最大接收量加1，只有空闲中断才能判断接收完成
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;						// 数据传输方向，从外设读取发送到内存
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								// 工作在正常模式，一次传输后自动结束
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;							// 高优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;						// 禁用FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;				// FIFO阈值为满
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			// 外设突发传输为单次
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					// 内存突发传输为单次
    DMA_Init(hw_info->dma_stream, &DMA_InitStructure);
    DMA_Cmd(hw_info->dma_stream, ENABLE);
	USART_DMACmd(cfg->uart_periph, USART_DMAReq_Rx, ENABLE);

#elif DRV_UART_PLATFORM_GD32F1
	dma_deinit(DMA0, hw_info->dma_channel);
	dma_parameter_struct dma_init_struct;
	dma_init_struct.periph_addr = cfg->uart_periph + 4;			// 外设基地址，数据寄存器偏移0x04
	dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;	// 外设数据宽度
	dma_init_struct.memory_addr = (uint32_t)cfg->rx_buf;		// 内存基地址
	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;		// 内存数据宽度
	dma_init_struct.number = RX_SINGLE_MAX + 1;					// 只有空闲中断才能判断接收完成，不会产生DMA完成中断
	dma_init_struct.priority = DMA_PRIORITY_HIGH;				// 优先级
	dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;	// 外设不递增
	dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;	// 内存递增
	dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;		// 从外设到内存
	dma_init(DMA0, hw_info->dma_channel, &dma_init_struct);

	dma_circulation_disable(DMA0, hw_info->dma_channel);
	dma_channel_enable(DMA0, hw_info->dma_channel);
	usart_dma_receive_config(cfg->uart_periph, USART_RECEIVE_DMA_ENABLE);
#endif
}

/**
 * @brief	串口格式化打印（硬件层实现）
 * @param[in] hw_info  硬件信息指针
 * @param[in] tx_buf   发送缓冲区
 * @param[in] buf_size 缓冲区大小
 * @param[in] format   格式化字符串
 * @param[in] args     可变参数列表
 */
static void uart_hw_printf(const uart_hw_info_t *hw_info, uint8_t *tx_buf, uint16_t buf_size, 
						   const char *format, va_list args)
{
    vsnprintf((char *)tx_buf, buf_size, format, args);

#if DRV_UART_PLATFORM_STM32F1 || DRV_UART_PLATFORM_STM32F4
    for (uint16_t i = 0; i < strlen((const char *)tx_buf); i++) {
        while (USART_GetFlagStatus(hw_info->uart_periph, USART_FLAG_TXE) == RESET);
        USART_SendData(hw_info->uart_periph, tx_buf[i]);
    }
    while (USART_GetFlagStatus(hw_info->uart_periph, USART_FLAG_TC) == RESET);

#elif DRV_UART_PLATFORM_GD32F1
	for (uint16_t i = 0; i < strlen((const char *)tx_buf); i++) {
		while (usart_flag_get(hw_info->uart_periph, USART_FLAG_TBE) == RESET);
		usart_data_transmit(hw_info->uart_periph, tx_buf[i]);
	}
	while (usart_flag_get(hw_info->uart_periph, USART_FLAG_TC) == RESET);
#endif
}

/**
 * @brief	串口数据发送（硬件层实现）
 * @param[in] hw_info uart_hw_info_t 结构体指针
 * @param[in] data    待发送数据
 * @param[in] len     数据长度（字节）
 */
static void uart_hw_send(const uart_hw_info_t *hw_info, uint8_t *data, uint32_t len)
{
#if DRV_UART_PLATFORM_STM32F1 || DRV_UART_PLATFORM_STM32F4
	for (uint32_t i = 0; i < len; i++) {
		USART_SendData(hw_info->uart_periph, data[i]);		// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (USART_GetFlagStatus(hw_info->uart_periph, USART_FLAG_TXE) == RESET);	// 等待发送完成
	}

#elif DRV_UART_PLATFORM_GD32F1
	for (uint32_t i = 0; i < len; i++) {
		usart_data_transmit(hw_info->uart_periph, data[i]);	// 将字节数据写入数据寄存器，写入后USART自动生成时序波形
		while (usart_flag_get(hw_info->uart_periph, USART_FLAG_TBE) == RESET);	// 等待发送完成
	}
#endif
}

/**
 * @brief	检查串口空闲中断标志
 * @param[in] hw_info uart_hw_info_t 结构体指针
 * @return	true 表示空闲中断触发，false 表示未触发
 */
static inline bool uart_hw_get_it_idle_flag(const uart_hw_info_t *hw_info)
{
#if DRV_UART_PLATFORM_STM32F1 || DRV_UART_PLATFORM_STM32F4
	return (bool)USART_GetITStatus(hw_info->uart_periph, USART_IT_IDLE);

#elif DRV_UART_PLATFORM_GD32F1
	return (bool)usart_interrupt_flag_get(hw_info->uart_periph, USART_INT_FLAG_IDLE);
#endif
}

/**
 * @brief	清除串口空闲中断标志
 * @param[in] hw_info uart_hw_info_t 结构体指针
 */
static inline void uart_hw_clear_it_flag(const uart_hw_info_t *hw_info)
{
#if DRV_UART_PLATFORM_STM32F1 || DRV_UART_PLATFORM_STM32F4
	volatile uint8_t clear;
	clear = hw_info->uart_periph->SR;
	clear = hw_info->uart_periph->DR;

#elif DRV_UART_PLATFORM_GD32F1
	usart_flag_get(hw_info->uart_periph, USART_FLAG_IDLEF);
	usart_data_receive(hw_info->uart_periph);
#endif
}

/**
 * @brief	获取 DMA 当前剩余数据计数
 * @param[in] hw_info uart_hw_info_t 结构体指针
 * @return	DMA 缓冲区剩余未传输的字节数
 */
static inline uint16_t uart_hw_dma_get_curr_data_counter(const uart_hw_info_t *hw_info)
{
#if DRV_UART_PLATFORM_STM32F1
	return DMA_GetCurrDataCounter(hw_info->dma_channel);
#elif DRV_UART_PLATFORM_STM32F4
	return DMA_GetCurrDataCounter(hw_info->dma_stream);
#elif DRV_UART_PLATFORM_GD32F1
	return dma_transfer_number_get(DMA0, hw_info->dma_channel);
#endif
}

/**
 * @brief	重新配置 DMA 接收
 * @param[in] hw_info   uart_hw_info_t 结构体指针
 * @param[in] buf_start 新的接收缓冲区起始地址
 * @param[in] buf_len   新的缓冲区长度
 */
static inline void uart_hw_dma_rx_reconfig(const uart_hw_info_t *hw_info,
                                           uint8_t *buf_start, uint16_t buf_len)
{
#if DRV_UART_PLATFORM_STM32F1
	dma_channel_t channel = hw_info->dma_channel;
	DMA_Cmd(channel, DISABLE);				// 关闭DMA
	while(channel->CCR & DMA_CCR1_EN);		// 等待DMA真正关闭
	channel->CNDTR = buf_len;				// 设置数据长度
	channel->CMAR = (uint32_t)buf_start;	// 设置内存地址
	DMA_Cmd(channel, ENABLE);				// 开启DMA

#elif DRV_UART_PLATFORM_STM32F4
	dma_stream_t stream = hw_info->dma_stream;
	DMA_Cmd(stream, DISABLE);						// 关闭DMA
	while(stream->CR & DMA_SxCR_EN);				// 等待DMA真正关闭
	stream->NDTR = buf_len;							// 设置数据长度
	stream->M0AR = (uint32_t)buf_start;				// 设置内存地址
	DMA_ClearFlag(stream, hw_info->dma_tcif_flag);	// 清除DMA传输完成中断标志位
	DMA_Cmd(stream, ENABLE);						// 开启DMA

#elif DRV_UART_PLATFORM_GD32F1
	dma_channel_t channel = hw_info->dma_channel;
	dma_channel_disable(DMA0, channel);
	dma_transfer_number_config(DMA0, channel, buf_len);
	dma_memory_address_config(DMA0, channel, (uint32_t)buf_start);
	dma_channel_enable(DMA0, channel);
#endif
}

/**
 * @brief   初始化串口硬件
 * @param[in] cfg uart_cfg_t 结构体指针
 */
static void uart_hw_init(const uart_cfg_t *cfg)
{
	uart_hw_gpio_clock_enable(cfg->tx_port);
	uart_hw_gpio_clock_enable(cfg->rx_port);
	uart_hw_uart_clock_enable(cfg->uart_periph);
	uart_hw_dma_clock_enable(cfg->uart_periph);

	uart_hw_gpio_init(cfg);
	uart_hw_uart_init(cfg);
	uart_hw_dma_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

#define IDX_BUF_NUM		10

/* 串口接收索引结构体 */
typedef struct {
    uint8_t *start;	// 数据段起始地址
    uint8_t *end;	// 数据段结束地址
} uart_rx_idx_t;

/* 串口接收控制块结构体 */
typedef struct {
    uint16_t 	   data_cnt;				// 已接收数据计数
    uart_rx_idx_t  idx_buf[IDX_BUF_NUM];	// 索引数组，用于管理多段数据
    uart_rx_idx_t *idx_in;					// 写入指针
    uart_rx_idx_t *idx_out;					// 读出指针
    uart_rx_idx_t *idx_end;					// 索引数组结束位置
} uart_rx_cb_t;

/* 私有数据结构体 */
typedef struct {
	uart_rx_cb_t rx_cb;
	uart_dev_t  *dev;
	bool 		 in_use;
} uart_priv_t;

static uart_priv_t g_uart_priv[MAX_UART_NUM];

static uart_priv_t *uart_priv_alloc(uart_periph_t uart_periph);
static void uart_priv_free(uart_priv_t *priv);
static char *uart_recv(uart_dev_t *dev);
static int uart_deinit_impl(uart_dev_t *dev);
static void uart_idle_irq_handler(uart_periph_t uart_periph);

/**
 * @brief	定义串口操作函数宏
 * @details 为指定串口生成 printf、send、recv 等操作函数及接口表
 * @param[in] x           串口号（如1、2、3）
 * @param[in] uart_periph 串口外设
 */
#define DEFINE_UART_OPS(x, uart_periph) 							\
static void uart##x##_printf_impl(char *format, ...)				\
{																	\
	const uart_hw_info_t *hw_info = uart_get_hw_info(uart_periph);  \
	uart_dev_t *dev = g_uart_priv[hw_info->idx].dev;				\
	va_list args;													\
	va_start(args, format);											\
	uart_hw_printf(hw_info, dev->cfg.tx_buf, 						\
				   dev->cfg.tx_buf_size, format, args); 			\
	va_end(args);													\
}																	\
static void uart##x##_send_impl(uint8_t *data, uint32_t len)   		\
{															   	    \
	const uart_hw_info_t *hw_info = uart_get_hw_info(uart_periph);  \
	uart_hw_send(hw_info, data, len);								\
}																	\
static char *uart##x##_recv_impl(void)								\
{															   	    \
	const uart_hw_info_t *hw_info = uart_get_hw_info(uart_periph);  \
	uart_dev_t *dev = g_uart_priv[hw_info->idx].dev;				\
	return uart_recv(dev);								   			\
}																	\
static const uart_ops_t uart##x##_ops = { 							\
    .printf = uart##x##_printf_impl, 								\
    .send = uart##x##_send_impl, 									\
    .recv = uart##x##_recv_impl, 									\
    .deinit = uart_deinit_impl 										\
}

/* 生成各平台的串口操作函数与接口表 */
#if DRV_UART_PLATFORM_STM32F1
DEFINE_UART_OPS(1, USART1);
DEFINE_UART_OPS(2, USART2);
DEFINE_UART_OPS(3, USART3);
#if defined(STM32F10X_HD)
DEFINE_UART_OPS(4, UART4);
#endif
#endif	/* DRV_UART_PLATFORM_STM32F1 */

#if DRV_UART_PLATFORM_STM32F4
#if defined(STM32F40_41xxx) || defined(STM32F429_439xx)
DEFINE_UART_OPS(1, USART1);
DEFINE_UART_OPS(2, USART2);
DEFINE_UART_OPS(3, USART3);
DEFINE_UART_OPS(4, UART4);
DEFINE_UART_OPS(5, UART5);
DEFINE_UART_OPS(6, USART6);
#elif defined(STM32F411xE)
DEFINE_UART_OPS(1, USART1);
DEFINE_UART_OPS(2, USART2);
DEFINE_UART_OPS(6, USART6);
#endif
#endif	/* DRV_UART_PLATFORM_STM32F4 */

#if DRV_UART_PLATFORM_GD32F1
DEFINE_UART_OPS(0, USART0);
DEFINE_UART_OPS(1, USART1);
DEFINE_UART_OPS(2, USART2);
#endif	/* DRV_UART_PLATFORM_GD32F1 */

/**
 * @brief   初始化串口设备驱动
 * @details 只能配置支持 DMA 的串口，配置为 DMA 接收
 * @param[out] dev uart_dev_t 结构体指针
 * @param[in]  cfg uart_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */											
int drv_uart_init(uart_dev_t *dev, const uart_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	uart_priv_t *priv = uart_priv_alloc(cfg->uart_periph);
	if (!priv)
		return -ENOMEM;

	/* 初始化接收数据控制块 */
	priv->rx_cb.idx_in = &priv->rx_cb.idx_buf[0];
	priv->rx_cb.idx_out = &priv->rx_cb.idx_buf[0];
	priv->rx_cb.idx_end = &priv->rx_cb.idx_buf[IDX_BUF_NUM - 1];
	priv->rx_cb.idx_in->start = cfg->rx_buf;
	priv->rx_cb.data_cnt = 0;

	priv->dev = dev;
	dev->priv = priv;
    dev->cfg  = *cfg;

	/* 关联操作接口表 */
	const uart_ops_t *uart_ops_table[MAX_UART_NUM] = {
#if defined(STM32F10X_MD)
	&uart1_ops, &uart2_ops, &uart3_ops
#elif defined(STM32F10X_HD)
	&uart1_ops, &uart2_ops, &uart3_ops, &uart4_ops
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
	&uart1_ops, &uart2_ops, &uart3_ops, &uart4_ops, &uart5_ops, &uart6_ops
#elif defined(STM32F411xE)
	&uart1_ops, &uart2_ops, &uart6_ops
#elif DRV_UART_PLATFORM_GD32F1
	&uart0_ops, &uart1_ops, &uart2_ops
#endif
	};
	
	const uart_hw_info_t *hw_info = uart_get_hw_info(cfg->uart_periph);
	dev->ops = uart_ops_table[hw_info->idx];

	/* 初始化硬件 */
	uart_hw_init(cfg);

	return 0;
}

/**
 * @brief   根据索引从私有数据数组中分配一个空闲槽位
 * @param[in] uart_periph 串口外设
 * @return	成功返回槽位指针，失败返回 NULL
 */
static uart_priv_t *uart_priv_alloc(uart_periph_t uart_periph)
{
	const uart_hw_info_t *hw_info = uart_get_hw_info(uart_periph);
    if (!hw_info)
		return NULL;
    
    uint8_t idx = hw_info->idx;
    if (g_uart_priv[idx].in_use)
		return NULL;
    
    g_uart_priv[idx].in_use = true;
    return &g_uart_priv[idx];
}

/**
 * @brief   释放私有数据槽位
 * @param[in,out] priv 待释放的槽位 uart_priv_t 结构体指针
 */
static void uart_priv_free(uart_priv_t *priv)
{
	if (priv)
		priv->in_use = false;
}

/**
 * @brief   串口通用接收函数，内部使用
 * @param[in] dev  uart_priv_t 结构体指针
 * @return	接收到数据字符串的首地址，未接收到数据则为 NULL
 */
static char *uart_recv(uart_dev_t *dev)
{
    if (!dev)
        return NULL;
    
    uart_priv_t *priv = (uart_priv_t *)dev->priv;

	/* 缓冲区中有未处理数据 */
    if (priv->rx_cb.idx_in != priv->rx_cb.idx_out) {
        char *start = (char *)priv->rx_cb.idx_out->start;
        size_t len = priv->rx_cb.idx_out->end - priv->rx_cb.idx_out->start + 1;

        /* 安全复制出数据（防止越界） */
        static char rx_buf_copy[RX_SINGLE_MAX + 1];
        if (len >= sizeof(rx_buf_copy))
            len = sizeof(rx_buf_copy) - 1;
        memcpy(rx_buf_copy, start, len);
        rx_buf_copy[len] = '\0';

        /* 处理完当前数据段后，移动idx_out指针到下一个位置，准备处理下一段数据 */
        priv->rx_cb.idx_out++;
        if (priv->rx_cb.idx_out == priv->rx_cb.idx_end)
            priv->rx_cb.idx_out = &priv->rx_cb.idx_buf[0];

        return rx_buf_copy;
    }

    return NULL;
}


/**
 * @brief   去初始化串口
 * @param[in,out] dev uart_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
static int uart_deinit_impl(uart_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	uart_priv_t *priv = (uart_priv_t *)dev->priv;
	uart_priv_free(priv);
	dev->priv = NULL;
	dev->ops = NULL;
	return 0;
}

/**
 * @brief   串口通用空闲中断函数，内部使用
 * @param[in] uart_periph 串口外设
 */	
static void uart_idle_irq_handler(uart_periph_t uart_periph)
{
	const uart_hw_info_t *hw_info = uart_get_hw_info(uart_periph);
    uart_priv_t *priv = &g_uart_priv[hw_info->idx];
	uart_dev_t *dev = priv->dev;

    if (uart_hw_get_it_idle_flag(hw_info)) {	// 检查空闲中断标志
		uart_hw_clear_it_flag(hw_info);			// 清除空闲中断标志

		/* 计算已接收数据量 */
		priv->rx_cb.data_cnt += (RX_SINGLE_MAX + 1) -
			uart_hw_dma_get_curr_data_counter(hw_info);

		/* 标记这一段数据的结尾 */
		priv->rx_cb.idx_in->end = &dev->cfg.rx_buf[priv->rx_cb.data_cnt - 1];

		/* 接收数据后in指针向后移动一位 */
		priv->rx_cb.idx_in++;
		if (priv->rx_cb.idx_in == priv->rx_cb.idx_end)
			priv->rx_cb.idx_in = &priv->rx_cb.idx_buf[0];

		/* 判断数据缓冲区剩余大小 */
		if (dev->cfg.rx_buf_size - priv->rx_cb.data_cnt >= RX_SINGLE_MAX) {
			/* 剩余大小足够再接收一次数据，标记下一段数据的start */
			priv->rx_cb.idx_in->start = &dev->cfg.rx_buf[priv->rx_cb.data_cnt];
		} else {
			/* 剩余大小不够再接收一次数据，回卷 */
			priv->rx_cb.idx_in->start = &dev->cfg.rx_buf[0];
			priv->rx_cb.data_cnt = 0;
		}

		/* 重新配置 DMA ，准备下一次接收 */
		uart_hw_dma_rx_reconfig(hw_info, priv->rx_cb.idx_in->start, RX_SINGLE_MAX + 1);
	}
}

/* 各串口中断服务函数 */
#if defined(USART0)
void USART0_IRQHandler(void) { uart_idle_irq_handler(USART0); }
#endif

#if defined(USART1)
void USART1_IRQHandler(void) { uart_idle_irq_handler(USART1); }
#endif

#if defined(USART2)
void USART2_IRQHandler(void) { uart_idle_irq_handler(USART2); }
#endif

#if defined(USART3)
void USART3_IRQHandler(void) { uart_idle_irq_handler(USART3); }
#endif

#if defined(UART4)
void UART4_IRQHandler(void)  { uart_idle_irq_handler(UART4);  }
#endif

#if defined(UART5)
void UART5_IRQHandler(void)  { uart_idle_irq_handler(UART5);  }
#endif

#if defined(USART6)
void USART6_IRQHandler(void) { uart_idle_irq_handler(USART6); }
#endif

/* ------------------------------- 核心驱动层结束 ------------------------------- */
