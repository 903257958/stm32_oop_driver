#include "main.h"

static uint8_t uart1_tx_buf[256];
static uint8_t uart1_rx_buf[256];
uart_dev_t debug = {
    .config = {
        .uartx          = USART1,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_Pin_9,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_Pin_10,
        .tx_buf         = uart1_tx_buf,
        .rx_buf         = uart1_rx_buf,
        .tx_buf_size    = sizeof(uart1_tx_buf),
        .rx_buf_size    = sizeof(uart1_rx_buf),
        .rx_single_max  = 64
    }
};

/* CAN1过滤器 */
/* 32Bit：11位STID、18位EXID、1位IDE、1位RTR、1位0 */
/* 16Bit：11位STID、1位RTR、1位IDE、3位0 */
can_filter_config_t can1_filters[] = {
    // /* 32位屏蔽模式示例，接收所有帧 */
    // {
    //     .id = 0,
    //     .mode = CAN_FILTER_MODE_32BIT_MASK,
    //     .id_high = 0x0000,
    //     .id_low  = 0x0000,
    //     .mask_id_high = 0x0000,
    //     .mask_id_low  = 0x0000,
    //     .fifo_assignment = CAN_Filter_FIFO0
    // },

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
    //     {
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
    {
        .id = 6,
        .mode = CAN_FILTER_MODE_32BIT_MASK,
        .id_high = 0x0 >> 16,
        .id_low  = 0x0,
        .mask_id_high = (0x01 << 1) >> 16,
        .mask_id_low  = (uint16_t)(0x01 << 1),
        .fifo_assignment = CAN_Filter_FIFO0
    },
};

/* CAN1设备 */
can_dev_t can1 = {
    .config = {
        .canx = CAN1,
        .rx_port = GPIOA,
        .rx_pin = GPIO_Pin_11,
        .tx_port = GPIOA,
        .tx_pin = GPIO_Pin_12,
        .mode = CAN_Mode_LoopBack,
        .filter_list = can1_filters,
        .filter_num = sizeof(can1_filters) / sizeof(can1_filters[0])
    }
};

can_tx_msg_t tx_msg_buf[] = {

    /* 测试 */
/*   StdId  ExtId       IDE              RTR            DLC   Data[8]                */
    // {0x555, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x000, 0x12345678, CAN_Id_Extended, CAN_RTR_Data,   4,   {0xAA, 0xBB, 0xCC, 0xDD}},    // 扩展格式数据帧
    // {0x666, 0x00000000, CAN_Id_Standard, CAN_RTR_Remote, 0,   NULL},                        // 标准格式遥控帧
    // {0x000, 0x0789ABCD, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧

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
    {0x123, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    {0x234, 0x00000000, CAN_Id_Standard, CAN_RTR_Remote, 0,   NULL},                        // 标准格式遥控帧
    {0x345, 0x00000000, CAN_Id_Standard, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    {0x456, 0x00000000, CAN_Id_Standard, CAN_RTR_Remote, 4,   NULL},                        // 标准格式遥控帧

    {0x000, 0x12345600, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    {0x000, 0x12345601, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧
    {0x000, 0x123456FE, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    {0x000, 0x123456FF, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧

    {0x000, 0x0789AB00, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    {0x000, 0x0789AB01, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧
    {0x000, 0x0789ABFE, CAN_Id_Extended, CAN_RTR_Data,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    {0x000, 0x0789ABFF, CAN_Id_Extended, CAN_RTR_Remote, 0,   NULL},                        // 扩展格式遥控帧
};
uint8_t tx_msg_index = 0;

can_rx_msg_t rx_msg;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	uart_init(&debug);
    can_init(&can1);

    debug.printf("\r\nCAN bus test!\r\n\r\n");
    
	while (1)
	{
        /* 发送 */
        can1.send(&can1, &tx_msg_buf[tx_msg_index++]);
        if (tx_msg_index >= sizeof(tx_msg_buf) / sizeof(can_tx_msg_t))
        {
            tx_msg_index = 0;
        }
        
        delay_ms(200);

        /* 接收 */
        if (can1.recv_flag(&can1, 0))
        {
            can1.recv(&can1, 0, &rx_msg);

            debug.printf("Rx: %s %s\r\n", 
                        ((rx_msg.IDE == CAN_Id_Standard) ? "Std" : "Ext"), 
                        ((rx_msg.RTR == CAN_RTR_Data) ? "Data" : "Remote"));


            if (rx_msg.IDE == CAN_Id_Standard)
            {
                debug.printf("ID: 0x%x,      ",  rx_msg.StdId);
            }
            else if (rx_msg.IDE == CAN_Id_Extended)
            {
                debug.printf("ID: 0x%08x, ",  rx_msg.ExtId);
            }

            debug.printf("Len: %d, ", rx_msg.DLC);

            if (rx_msg.RTR == CAN_RTR_Data)
            {
                debug.printf("Data: 0x%x 0x%x 0x%x 0x%x\r\n\r\n", 
                            rx_msg.Data[0], rx_msg.Data[1], rx_msg.Data[2], rx_msg.Data[3]);
            }
            else if (rx_msg.RTR == CAN_RTR_Remote)
            {
                debug.printf("Data: NULL\r\n\r\n");
            }
        }

		delay_ms(200);
	}
}
