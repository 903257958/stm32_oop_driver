#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_spi.h"
#include "drv_w25qx.h"

static uart_dev_t uart_debug;
static uint8_t uart_debug_tx_buf[256];
static uint8_t uart_debug_rx_buf[256];
static const uart_cfg_t uart_debug_cfg = {
    .uart_periph     = USART1,
    .baud            = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_9,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_10,
    .tx_buf          = uart_debug_tx_buf,
    .rx_buf          = uart_debug_rx_buf,
    .tx_buf_size     = sizeof(uart_debug_tx_buf),
    .rx_buf_size     = sizeof(uart_debug_rx_buf),
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static spi_dev_t spi2;
static const spi_cfg_t spi2_cfg = {
	.spi_periph = SPI2,
	.sck_port   = GPIOB,
	.sck_pin    = GPIO_Pin_10,
	.miso_port  = GPIOB,
	.miso_pin   = GPIO_Pin_14,
	.mosi_port  = GPIOB,
	.mosi_pin   = GPIO_Pin_15,
	.prescaler  = SPI_BaudRatePrescaler_2,
	.mode       = SPI_MODE_0,
};

static int spi2_start(gpio_port_t cs_port, gpio_pin_t cs_pin)
{
	return spi2.ops->start(&spi2, cs_port, cs_pin);
}
static int spi2_swap_byte(uint8_t send, uint8_t *recv)
{
	return spi2.ops->swap_byte(&spi2, send, recv);
}
static int spi2_stop(gpio_port_t cs_port, gpio_pin_t cs_pin)
{
	return spi2.ops->stop(&spi2, cs_port, cs_pin);
}
static w25qx_spi_ops_t w25qx_spi_ops = {
	.start     = spi2_start,
	.swap_byte = spi2_swap_byte,
	.stop      = spi2_stop	
};

static w25qx_dev_t w25q128;
static const w25qx_cfg_t w25q128_cfg = {
	.spi_ops = &w25qx_spi_ops,
	.cs_port = GPIOC,
	.cs_pin  = GPIO_Pin_12
};

int main(void)
{
    uint8_t mid;
	uint16_t did;
    uint8_t write_buf[W25QX_PAGE_SIZE];
    uint8_t read_buf[W25QX_PAGE_SIZE];
    uint32_t i;
    uint32_t error_cnt;
    int ret;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_spi_init(&spi2, &spi2_cfg);
    ret = drv_w25qx_init(&w25q128, &w25q128_cfg);
    
    uart_debug.ops->printf("=== W25QX Function Test Start ===\r\n");
    uart_debug.ops->printf("1. Initialization test: ");
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (ret=%d)\r\n", ret);
        goto test_end;
    }
    uart_debug.ops->printf("OK\r\n");

    /* 2. 读取ID测试 */
    uart_debug.ops->printf("2. Read ID test: ");
    ret = w25q128.ops->read_id(&w25q128, &mid, &did);
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (ret=%d)\r\n", ret);
        goto test_end;
    }
    uart_debug.ops->printf("OK (mid:0x%02X, did:0x%04X)\r\n", mid, did);

    /* 3. 唤醒测试 */
    uart_debug.ops->printf("3. Wakeup test: ");
    ret = w25q128.ops->wakeup(&w25q128);
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (ret=%d)\r\n", ret);
        goto test_end;
    }
    uart_debug.ops->printf("OK\r\n");

    /* 4. 64KB块擦除测试 */
    uart_debug.ops->printf("4. 64KB Block Erase test: ");
    ret = w25q128.ops->erase_block_64kb(&w25q128, 0);
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (ret=%d)\r\n", ret);
        goto test_end;
    }
    /* 验证擦除结果（应为0xFF） */
    w25q128.ops->read_data(&w25q128, 0, W25QX_PAGE_SIZE, read_buf);
    for (i = 0; i < W25QX_PAGE_SIZE; i++) {
        if (read_buf[i] != 0xFF) {
            uart_debug.ops->printf("FAILED (erase incomplete)\r\n");
            goto test_end;
        }
    }
    uart_debug.ops->printf("OK\r\n");

    /* 5. 页写入测试 */
    uart_debug.ops->printf("5. Page Write test: ");
    for (i = 0; i < W25QX_PAGE_SIZE; i++) {
        write_buf[i] = i;  // 填充测试数据
    }
    ret = w25q128.ops->write_page(&w25q128, 0, W25QX_PAGE_SIZE, write_buf);
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (ret=%d)\r\n", ret);
        goto test_end;
    }
    /* 验证页写入 */
    w25q128.ops->read_data(&w25q128, 0, W25QX_PAGE_SIZE, read_buf);
    error_cnt = 0;
    for (i = 0; i < W25QX_PAGE_SIZE; i++) {
        if (read_buf[i] != write_buf[i])
			error_cnt++;
    }
    if (error_cnt != 0) {
        uart_debug.ops->printf("FAILED (errors=%d)\r\n", error_cnt);
        goto test_end;
    }
    uart_debug.ops->printf("OK\r\n");

    /* 6. 4KB扇区擦除测试 */
    uart_debug.ops->printf("6. 4KB Sector Erase test: ");
    ret = w25q128.ops->erase_sector_4kb(&w25q128, 0);
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (ret=%d)\r\n", ret);
        goto test_end;
    }
    /* 验证扇区擦除 */
    w25q128.ops->read_data(&w25q128, 0, W25QX_PAGE_SIZE, read_buf);
    for (i = 0; i < W25QX_PAGE_SIZE; i++) {
        if (read_buf[i] != 0xFF) {
            uart_debug.ops->printf("FAILED (erase incomplete)\r\n");
            goto test_end;
        }
    }
    uart_debug.ops->printf("OK\r\n");

    /* 7. 跨页写入测试 */
    uart_debug.ops->printf("7. Cross-page Write test: ");
    for (i = 0; i < W25QX_PAGE_SIZE * 2; i++) {
        write_buf[i % W25QX_PAGE_SIZE] = i;  // 生成跨页数据
    }
    /* 从页末尾开始写入，触发跨页操作 */
    ret = w25q128.ops->write_data(&w25q128, W25QX_PAGE_SIZE - 10, 20, write_buf);
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (ret=%d)\r\n", ret);
        goto test_end;
    }
    /* 验证跨页写入 */
    w25q128.ops->read_data(&w25q128, W25QX_PAGE_SIZE - 10, 20, read_buf);
    error_cnt = 0;
    for (i = 0; i < 20; i++) {
        if (read_buf[i] != i)
			error_cnt++;
    }
    if (error_cnt != 0) {
        uart_debug.ops->printf("FAILED (errors=%d)\r\n", error_cnt);
        goto test_end;
    }
    uart_debug.ops->printf("OK\r\n");

    /* 8. 去初始化测试 */
    uart_debug.ops->printf("8. Deinitialization test: ");
    ret = w25q128.ops->deinit(&w25q128);
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (ret=%d)\r\n", ret);
        goto test_end;
    }
    uart_debug.ops->printf("OK\r\n");

test_end:
    uart_debug.ops->printf("=== W25QX Function Test Complete ===\r\n\r\n");
    while (1) {
        
    }
}
