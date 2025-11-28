#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_can.h"
#include <stddef.h>

static uart_dev_t uart_debug;
static uint8_t uart_debug_tx_buf[512];
static uint8_t uart_debug_rx_buf[512];
static const uart_cfg_t uart_debug_cfg = {
    .uart_periph     = USART1,
    .baudrate        = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_9,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_10,
    .tx_buf          = uart_debug_tx_buf,
    .rx_buf          = uart_debug_rx_buf,
    .tx_buf_size     = sizeof(uart_debug_tx_buf),
    .rx_buf_size     = sizeof(uart_debug_rx_buf),
    .rx_single_max   = 256,
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static can_dev_t can1;
/* CAN1过滤器 */
/* 32Bit：11位STID、18位EXID、1位IDE、1位RTR、1位0 */
/* 16Bit：11位STID、1位RTR、1位IDE、3位0 */
static can_filter_cfg_t can1_filters[] = {
    /* 32位屏蔽模式示例，接收所有帧 */
    {
        .id = 0,
        .mode = CAN_FILTER_MODE_32BIT_MASK,
        .id_high = 0x0000,
        .id_low  = 0x0000,
        .mask_id_high = 0x0000,
        .mask_id_low  = 0x0000,
        .fifo_assignment = CAN_Filter_FIFO0
    },

    /* 16位列表模式示例，只接收标准格式0x234，0x345，0x567 */
	// {
    //     .id = 1,
    //     .mode = CAN_FILTER_MODE_16BIT_LIST,
    //     .id_high = (0x234 << 5) | (0x01 << 4),
    //     .id_low  = 0x345 << 5,
    //     .mask_id_high = 0x456 << 5,
    //     .mask_id_low  = 0x000 << 5,
    //     .fifo_assignment = CAN_Filter_FIFO0
    // },

    /* 16位屏蔽模式示例，只接收标准格式0x200~0x2FF，0x320~0x32F */
    // {
    //     .id = 2,
    //     .mode = CAN_FILTER_MODE_16BIT_MASK,
    //     .id_high = 0x234 << 5,
    //     .id_low  = 0x320 << 5,
    //     .mask_id_high = (0x700 << 5) | (0x01 << 4) | (0x01 << 3),
    //     .mask_id_low  = (0x7F0 << 5) | (0x01 << 4) | (0x01 << 3),
    //     .fifo_assignment = CAN_Filter_FIFO0
    // },

    /* 32位列表模式示例，只接收标准格式0x123和扩展格式0x12345678 */
    // {
    //     .id = 3,
    //     .mode = CAN_FILTER_MODE_32BIT_LIST,
    //     .id_high = (0x123U << 21) >> 16,
    //     .id_low  = (uint16_t)(0x123U << 21),
    //     .mask_id_high = ((0x12345678U << 3) | (0x01 << 2)) >> 16,
    //     .mask_id_low  = (uint16_t)((0x12345678U << 3) | (0x01 << 2)),
    //     .fifo_assignment = CAN_Filter_FIFO0
    // },

    /* 32位屏蔽模式示例，只接收扩展格式0x12345600~0x123456FF */
    // {
    //     .id = 4,
    //     .mode = CAN_FILTER_MODE_32BIT_MASK,
    //     .id_high = ((0x12345600U << 3) | (0x01 << 2)) >> 16,
    //     .id_low  = (uint16_t)((0x12345600U << 3) | (0x01 << 2)),
    //     .mask_id_high = ((0x1FFFFF00U << 3) | (0x01 << 2) | (0x01 << 1)) >> 16,
    //     .mask_id_low  = (uint16_t)((0x1FFFFF00U << 3) | (0x01 << 2) | (0x01 << 1)),
    //     .fifo_assignment = CAN_Filter_FIFO0
    // },

    /* 32位屏蔽模式示例，只接收遥控帧 */
    // {
    //     .id = 5,
    //     .mode = CAN_FILTER_MODE_32BIT_MASK,
    //     .id_high = (0x01 << 1) >> 16,
    //     .id_low  = (uint16_t)(0x01 << 1),
    //     .mask_id_high = (0x01 << 1) >> 16,
    //     .mask_id_low  = (uint16_t)(0x01 << 1),
    //     .fifo_assignment = CAN_Filter_FIFO0
    // },

    /* 32位屏蔽模式示例，只接收数据帧 */
    // {
    //     .id = 6,
    //     .mode = CAN_FILTER_MODE_32BIT_MASK,
    //     .id_high = 0x0 >> 16,
    //     .id_low  = 0x0,
    //     .mask_id_high = (0x01 << 1) >> 16,
    //     .mask_id_low  = (uint16_t)(0x01 << 1),
    //     .fifo_assignment = CAN_Filter_FIFO0
    // },
};
static const can_cfg_t can1_cfg = {
    .can_periph  = CAN1,
    .baudrate    = CAN_BAUDRATE_250K, 
    .rx_port     = GPIOA,
    .rx_pin      = GPIO_Pin_11,
    .tx_port     = GPIOA,
    .tx_pin      = GPIO_Pin_12,
    .mode        = CAN_Mode_LoopBack, /* or CAN_Mode_Normal */
    .filter_list = can1_filters,
    .filter_num  = sizeof(can1_filters) / sizeof(can1_filters[0])
};


static can_tx_msg_t tx_msg_buf[] = {

    /* 测试 */
/*   StdId  ExtId       IDE              RTR            DLC   Data[8]                */
    {0x555, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    {0x000, 0x12345678, CAN_Id_Extended, CAN_RTR_Data,   4,   {0xAA, 0xBB, 0xCC, 0xDD}},    // 扩展格式数据帧
    {0x666, 0x00000000, CAN_Id_Standard, CAN_RTR_Remote, 0,   NULL},                        // 标准格式遥控帧
    {0x000, 0x0789ABCD, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧

    /* 16位列表模式过滤器测试 */
/*   StdId  ExtId       IDE              RTR            DLC   Data[8]                */
    // {0x123, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x234, 0x00000000, CAN_Id_Standard, CAN_RTR_Remote, 4,   NULL},                        // 标准格式遥控帧
    // {0x345, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x456, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x567, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x678, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    /* 16位屏蔽模式过滤器测试 */
/*   StdId  ExtId       IDE              RTR            DLC   Data[8]                */
    // {0x100, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x101, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x1FE, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x1FF, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    // {0x200, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x201, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x2FE, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x2FF, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    // {0x300, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x301, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x3FE, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x3FF, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    // {0x320, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x321, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x32E, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x32F, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    /* 32位列表模式过滤器测试 */
/*   StdId  ExtId       IDE              RTR            DLC   Data[8]                */
    // {0x123, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x234, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式遥控帧
    // {0x345, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x456, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x000, 0x12345678, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789ABCD, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧

    /* 32位屏蔽模式过滤器测试 */
/*   StdId  ExtId       IDE              RTR            DLC   Data[8]                */
    // {0x000, 0x12345600, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x12345601, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x123456FE, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x123456FF, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧

    // {0x000, 0x0789AB00, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789AB01, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789ABFE, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789ABFF, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧

    /* 32位屏蔽模式过滤器测试（只接收遥控/数据帧） */
/*   StdId  ExtId       IDE              RTR            DLC   Data[8]                */
    // {0x123, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x234, 0x00000000, CAN_Id_Standard, CAN_RTR_Remote, 0,   NULL},                        // 标准格式遥控帧
    // {0x345, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x456, 0x00000000, CAN_Id_Standard, CAN_RTR_Remote, 4,   NULL},                        // 标准格式遥控帧

    // {0x000, 0x12345600, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x12345601, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧
    // {0x000, 0x123456FE, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x123456FF, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧

    // {0x000, 0x0789AB00, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789AB01, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧
    // {0x000, 0x0789ABFE, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789ABFF, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧
};
static uint8_t tx_msg_index = 0;
static can_rx_msg_t rx_msg;

int main(void)
{
	uint8_t i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_can_init(&can1, &can1_cfg);

	uart_debug.ops->printf(&uart_debug, "\r\nCAN bus test!\r\n\r\n");
	
	while (1) {
		/* 发送 */
        can1.ops->send(&can1, &tx_msg_buf[tx_msg_index]);
		uart_debug.ops->printf(&uart_debug, "Tx: %s %s 0x%08X [%d] {", 
                               (tx_msg_buf[tx_msg_index].IDE == CAN_Id_Standard) ? "[STD" : "[EXT", 
                               (tx_msg_buf[tx_msg_index].RTR == CAN_RTR_Data) ? "DATA]  " : "REMOTE]",
                               (tx_msg_buf[tx_msg_index].IDE == CAN_Id_Standard) ? 
                                tx_msg_buf[tx_msg_index].StdId : tx_msg_buf[tx_msg_index].ExtId,
                                tx_msg_buf[tx_msg_index].DLC);
			
		if (tx_msg_buf[tx_msg_index].RTR == CAN_RTR_Data)
			for (i = 0; i < tx_msg_buf[tx_msg_index].DLC; i++)
				uart_debug.ops->printf(&uart_debug, " 0x%02X", tx_msg_buf[tx_msg_index].Data[i]);
		uart_debug.ops->printf(&uart_debug, " }\r\n");
		
		tx_msg_index++;
        if (tx_msg_index >= sizeof(tx_msg_buf) / sizeof(can_tx_msg_t))
            tx_msg_index = 0;
		
        delay_ms(500);

        /* 接收 */
        if (can1.ops->recv_flag(&can1, CAN_FIFO0)) {
            can1.ops->recv(&can1, CAN_FIFO0, &rx_msg);

            uart_debug.ops->printf(&uart_debug, "Rx: %s %s 0x%08X [%d] {", 
                                   (rx_msg.IDE == CAN_Id_Standard) ? "[STD" : "[EXT", 
                                   (rx_msg.RTR == CAN_RTR_Data) ? "DATA]  " : "REMOTE]",
                                   (rx_msg.IDE == CAN_Id_Standard) ? rx_msg.StdId : rx_msg.ExtId,
                                    rx_msg.DLC);
			
			if (rx_msg.RTR == CAN_RTR_Data)
				for (i = 0; i < rx_msg.DLC; i++)
					uart_debug.ops->printf(&uart_debug, " 0x%02X", rx_msg.Data[i]);
			uart_debug.ops->printf(&uart_debug, " }\r\n\r\n");
        }

		delay_ms(500);
	}
}
