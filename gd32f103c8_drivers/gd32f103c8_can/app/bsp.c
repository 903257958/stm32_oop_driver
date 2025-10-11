#include "bsp.h"

/* 硬件设备定义 */
static uint8_t uart0_tx_buf[2048];
static uint8_t uart0_rx_buf[2048];
uart_dev_t debug = {
    .config = {
        .uartx          = USART0,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_PIN_9,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_PIN_10,
        .tx_buf         = uart0_tx_buf,
        .rx_buf         = uart0_rx_buf,
        .tx_buf_size    = sizeof(uart0_tx_buf),
        .rx_buf_size    = sizeof(uart0_rx_buf),
        .rx_single_max  = 512
    }
};

/* CAN0过滤器 */
/* 32Bit：11位STID、18位EXID、1位IDE、1位RTR、1位0 */
/* 16Bit：11位STID、1位RTR、1位IDE、3位0 */
static can_filter_config_t can0_filters[] = {
    /* 32位屏蔽模式示例，接收所有帧 */
    // {
    //     .id = 0,
    //     .mode = CAN_FILTER_MODE_32BIT_MASK,
    //     .id_high = 0x0000,
    //     .id_low  = 0x0000,
    //     .mask_id_high = 0x0000,
    //     .mask_id_low  = 0x0000,
    //     .fifo_assignment = CAN_FIFO0
    // },

    /* 16位列表模式示例，只接收标准格式0x234，0x345，0x567 */
	// {
    //     .id = 1,
    //     .mode = CAN_FILTER_MODE_16BIT_LIST,
    //     .id_high = (0x234 << 5) | (0x01 << 4),
    //     .id_low  = 0x345 << 5,
    //     .mask_id_high = 0x456 << 5,
    //     .mask_id_low  = 0x000 << 5,
    //     .fifo_assignment = CAN_FIFO0
    // },

    /* 16位屏蔽模式示例，只接收标准格式0x200~0x2FF，0x320~0x32F */
    // {
    //     .id = 2,
    //     .mode = CAN_FILTER_MODE_16BIT_MASK,
    //     .id_high = 0x234 << 5,
    //     .id_low  = 0x320 << 5,
    //     .mask_id_high = (0x700 << 5) | (0x01 << 4) | (0x01 << 3),
    //     .mask_id_low  = (0x7F0 << 5) | (0x01 << 4) | (0x01 << 3),
    //     .fifo_assignment = CAN_FIFO0
    // },

    /* 32位列表模式示例，只接收标准格式0x123和扩展格式0x12345678 */
    // {
    //     .id = 3,
    //     .mode = CAN_FILTER_MODE_32BIT_LIST,
    //     .id_high = (0x123U << 21) >> 16,
    //     .id_low  = (uint16_t)(0x123U << 21),
    //     .mask_id_high = ((0x12345678U << 3) | (0x01 << 2)) >> 16,
    //     .mask_id_low  = (uint16_t)((0x12345678U << 3) | (0x01 << 2)),
    //     .fifo_assignment = CAN_FIFO0
    // },

    /* 32位屏蔽模式示例，只接收扩展格式0x12345600~0x123456FF */
    // {
    //     .id = 4,
    //     .mode = CAN_FILTER_MODE_32BIT_MASK,
    //     .id_high = ((0x12345600U << 3) | (0x01 << 2)) >> 16,
    //     .id_low  = (uint16_t)((0x12345600U << 3) | (0x01 << 2)),
    //     .mask_id_high = ((0x1FFFFF00U << 3) | (0x01 << 2) | (0x01 << 1)) >> 16,
    //     .mask_id_low  = (uint16_t)((0x1FFFFF00U << 3) | (0x01 << 2) | (0x01 << 1)),
    //     .fifo_assignment = CAN_FIFO0
    // },

    /* 32位屏蔽模式示例，只接收遥控帧 */
    // {
    //     .id = 5,
    //     .mode = CAN_FILTER_MODE_32BIT_MASK,
    //     .id_high = (0x01 << 1) >> 16,
    //     .id_low  = (uint16_t)(0x01 << 1),
    //     .mask_id_high = (0x01 << 1) >> 16,
    //     .mask_id_low  = (uint16_t)(0x01 << 1),
    //     .fifo_assignment = CAN_FIFO0
    // },

    /* 32位屏蔽模式示例，只接收数据帧 */
    {
        .id = 6,
        .mode = CAN_FILTER_MODE_32BIT_MASK,
        .id_high = 0x0 >> 16,
        .id_low  = 0x0,
        .mask_id_high = (0x01 << 1) >> 16,
        .mask_id_low  = (uint16_t)(0x01 << 1),
        .fifo_assignment = CAN_FIFO0
    },
};

can_dev_t can0 = {
    .config = {
        .canx = CAN0,
        .rx_port = GPIOA,
        .rx_pin = GPIO_PIN_11,
        .tx_port = GPIOA,
        .tx_pin = GPIO_PIN_12,
        .mode = CAN_LOOPBACK_MODE,
        .filter_list = can0_filters,
        .filter_num = sizeof(can0_filters) / sizeof(can0_filters[0])
    }
};

/**
 * @brief   初始化 BSP 硬件
 * @return  0 表示成功
 */
int bsp_init(void)
{
    uart_init(&debug);
    can_drv_init(&can0);
    
    return 0;
}
