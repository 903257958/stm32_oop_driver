#include "bsp.h"

static can_tx_msg_t tx_msg_buf[] = {

    /* 测试 */
/*   rx_sfid  rx_efid       rx_ff              rx_ft            rx_dlen   Data[8]                */
    {0x555, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    {0x000, 0x12345678, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0xAA, 0xBB, 0xCC, 0xDD}},    // 扩展格式数据帧
    {0x666, 0x00000000, CAN_FF_STANDARD, CAN_FT_REMOTE, 0,   NULL},                        // 标准格式遥控帧
    {0x000, 0x0789ABCD, CAN_FF_EXTENDED, CAN_FT_REMOTE, 0,   NULL},                        // 扩展格式遥控帧

    /* 16位列表模式过滤器测试 */
/*   rx_sfid  rx_efid       rx_ff              rx_ft            rx_dlen   Data[8]                */
    // {0x123, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x234, 0x00000000, CAN_FF_STANDARD, CAN_FT_REMOTE, 4,   NULL},                        // 标准格式遥控帧
    // {0x345, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x456, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x567, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x678, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    /* 16位屏蔽模式过滤器测试 */
/*   rx_sfid  rx_efid       rx_ff              rx_ft            rx_dlen   Data[8]                */
    // {0x100, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x101, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x1FE, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x1FF, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    // {0x200, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x201, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x2FE, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x2FF, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    // {0x300, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x301, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x3FE, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x3FF, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    // {0x320, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x321, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x32E, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x32F, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧

    /* 32位列表模式过滤器测试 */
/*   rx_sfid  rx_efid       rx_ff              rx_ft            rx_dlen   Data[8]                */
    // {0x123, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x234, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式遥控帧
    // {0x345, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x456, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x000, 0x12345678, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789ABCD, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧

    /* 32位屏蔽模式过滤器测试 */
/*   rx_sfid  rx_efid       rx_ff              rx_ft            rx_dlen   Data[8]                */
    // {0x000, 0x12345600, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x12345601, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x123456FE, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x123456FF, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧

    // {0x000, 0x0789AB00, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789AB01, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789ABFE, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789ABFF, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧

    /* 32位屏蔽模式过滤器测试（只接收遥控/数据帧） */
/*   rx_sfid  rx_efid       rx_ff              rx_ft            rx_dlen   Data[8]                */
    // {0x123, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x234, 0x00000000, CAN_FF_STANDARD, CAN_FT_REMOTE, 0,   NULL},                        // 标准格式遥控帧
    // {0x345, 0x00000000, CAN_FF_STANDARD, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 标准格式数据帧
    // {0x456, 0x00000000, CAN_FF_STANDARD, CAN_FT_REMOTE, 0,   NULL},                        // 标准格式遥控帧

    // {0x000, 0x12345600, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x12345601, CAN_FF_EXTENDED, CAN_FT_REMOTE, 0,   NULL},                        // 扩展格式遥控帧
    // {0x000, 0x123456FE, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x123456FF, CAN_FF_EXTENDED, CAN_FT_REMOTE, 0,   NULL},                        // 扩展格式遥控帧

    // {0x000, 0x0789AB00, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789AB01, CAN_FF_EXTENDED, CAN_FT_REMOTE, 0,   NULL},                        // 扩展格式遥控帧
    // {0x000, 0x0789ABFE, CAN_FF_EXTENDED, CAN_FT_DATA,   4,   {0x11, 0x22, 0x33, 0x44}},    // 扩展格式数据帧
    // {0x000, 0x0789ABFF, CAN_FF_EXTENDED, CAN_FT_REMOTE, 0,   NULL},                        // 扩展格式遥控帧
};

static uint8_t tx_msg_index = 0;
static can_rx_msg_t rx_msg;

int main(void)
{
	uint8_t i;

	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    bsp_init();

	debug.printf("\r\nCAN bus test!\r\n\r\n");
	
	while (1) {
		/* 发送 */
        can0.send(&can0, &tx_msg_buf[tx_msg_index]);
		debug.printf("Tx: %s %s 0x%08X [%d] {", 
					(tx_msg_buf[tx_msg_index].tx_ff == CAN_FF_STANDARD) ? "[STD" : "[EXT", 
					(tx_msg_buf[tx_msg_index].tx_ft == CAN_FT_DATA) ? "DATA]  " : "REMOTE]",
					(tx_msg_buf[tx_msg_index].tx_ff == CAN_FF_STANDARD) ? 
					tx_msg_buf[tx_msg_index].tx_sfid : tx_msg_buf[tx_msg_index].tx_efid,
					tx_msg_buf[tx_msg_index].tx_dlen);
			
		if (tx_msg_buf[tx_msg_index].tx_ft == CAN_FT_DATA)
			for (i = 0; i < tx_msg_buf[tx_msg_index].tx_dlen; i++)
				debug.printf(" 0x%02X", tx_msg_buf[tx_msg_index].tx_data[i]);
		debug.printf(" }\r\n");
		
		tx_msg_index++;
        if (tx_msg_index >= sizeof(tx_msg_buf) / sizeof(can_tx_msg_t))
            tx_msg_index = 0;
		
        delay_ms(500);

        /* 接收 */
        if (can0.recv_flag(&can0, CAN_FIFO0)) {
            can0.recv(&can0, CAN_FIFO0, &rx_msg);

            debug.printf("Rx: %s %s 0x%08X [%d] {", 
                        (rx_msg.rx_ff == CAN_FF_STANDARD) ? "[STD" : "[EXT", 
                        (rx_msg.rx_ft == CAN_FT_DATA) ? "DATA]  " : "REMOTE]",
						(rx_msg.rx_ff == CAN_FF_STANDARD) ? rx_msg.rx_sfid : rx_msg.rx_efid,
						rx_msg.rx_dlen);
			
			if (rx_msg.rx_ft == CAN_FT_DATA)
				for (i = 0; i < rx_msg.rx_dlen; i++)
					debug.printf(" 0x%02X", rx_msg.rx_data[i]);
			debug.printf(" }\r\n\r\n");
        }

		delay_ms(500);
	}
}
