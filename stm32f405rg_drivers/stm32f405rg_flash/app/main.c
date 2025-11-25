#include "drv_uart.h"
#include "drv_flash.h"
#include <string.h>

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
};

static flash_dev_t flash;
static uint32_t test_addr;
static uint32_t write_buf[16], read_buf[16];

int main(void)
{
    uint32_t i, error_cnt = 0;
    int ret;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_flash_init(&flash);
    uart_debug.ops->printf("\r\n=== Internal Flash Test ===\r\n");

    // 初始化测试数据
    for (i = 0; i < 16; i++)
        write_buf[i] = 0xAA55AA00 + i;

    // 选择测试地址（根据平台选择安全区域）
    #if DRV_FLASH_PLATFORM_STM32F1 || DRV_FLASH_PLATFORM_GD32F1
    test_addr = 0x08000000 + 60 * 1024; // 页地址
    #elif DRV_FLASH_PLATFORM_STM32F4
    test_addr = 0x080E0000; // 扇区地址
    #endif

    // 1. 擦除测试
    uart_debug.ops->printf("1. Erase test: ");
    #if DRV_FLASH_PLATFORM_STM32F1 || DRV_FLASH_PLATFORM_GD32F1
    ret = flash.ops->page_erase(&flash, 1, 60);
    #else
    ret = flash.ops->sector_erase(&flash, 1, 11);
    #endif
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (erase ret=%d)\r\n", ret);
        goto end;
    }
    // 验证擦除（应为0xFFFFFFFF）
    for (i = 0; i < 16; i++) {
        if (*(uint32_t*)(test_addr + i*4) != 0xFFFFFFFF)
            error_cnt++;
    }
    if (error_cnt != 0) {
        uart_debug.ops->printf("FAILED (erase verify err=%d)\r\n", error_cnt);
        goto end;
    }
    uart_debug.ops->printf("OK\r\n");

    // 2. 写入测试
    uart_debug.ops->printf("2. Write test: ");
    ret = flash.ops->write(&flash, test_addr, 64, write_buf);
    if (ret != 0) {
        uart_debug.ops->printf("FAILED (write ret=%d)\r\n", ret);
        goto end;
    }
    uart_debug.ops->printf("OK\r\n");

    // 3. 读取验证
    uart_debug.ops->printf("3. Read verify: ");
    error_cnt = 0;
    for (i = 0; i < 16; i++) {
        read_buf[i] = *(uint32_t*)(test_addr + i*4);
        if (read_buf[i] != write_buf[i])
            error_cnt++;
    }
    if (error_cnt != 0) {
        uart_debug.ops->printf("FAILED (mismatch=%d)\r\n", error_cnt);
        goto end;
    }
    uart_debug.ops->printf("OK\r\n");

end:
    uart_debug.ops->printf("=== Test Complete ===\r\n\r\n");
    while (1);
}
