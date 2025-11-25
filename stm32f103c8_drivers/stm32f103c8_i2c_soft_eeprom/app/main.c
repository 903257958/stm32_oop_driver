#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_i2c_soft.h"
#include "drv_eeprom.h"
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
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static i2c_soft_dev_t i2c_soft;
static const i2c_soft_cfg_t i2c_soft_cfg = {
    .scl_port     = GPIOB, 
    .scl_pin      = GPIO_Pin_6, 
    .sda_port     = GPIOB, 
    .sda_pin      = GPIO_Pin_7,
    .delay_us     = delay_us, 
    .bit_delay_us = 1
};

int i2c_soft_write_reg_adapter(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return i2c_soft.ops->write_reg(&i2c_soft, dev_addr, reg_addr, data);
}

int i2c_soft_write_regs_adapter(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data)
{
    return i2c_soft.ops->write_regs(&i2c_soft, dev_addr, reg_addr, num, data);
}

int i2c_soft_read_regs_adapter(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data)
{
    return i2c_soft.ops->read_regs(&i2c_soft, dev_addr, reg_addr, num, data);
}

static eeprom_i2c_ops_t eeprom_i2c_ops = {
    .write_reg  = i2c_soft_write_reg_adapter, 
    .write_regs = i2c_soft_write_regs_adapter,
    .read_regs  = i2c_soft_read_regs_adapter
};

static eeprom_dev_t at24c02;
static const eeprom_cfg_t at24c02_cfg = {
    .i2c_ops   = &eeprom_i2c_ops, 
    .page_size = EEPROM_AT24C02_PAGE_SIZE
};

int main(void)
{
    uint8_t rbuf[EEPROM_AT24C02_PAGE_SIZE + 2];
    uint8_t wbuf[EEPROM_AT24C02_PAGE_SIZE] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
    int ret;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_i2c_soft_init(&i2c_soft, &i2c_soft_cfg);
    drv_eeprom_init(&at24c02, &at24c02_cfg);

    uart_debug.ops->printf("==== EEPROM Test Start ====\r\n");

    /* 1. 单字节写读测试 */
    ret = at24c02.ops->write_byte(&at24c02, 0x00, 0xAB);
    delay_ms(5);
    ret |= at24c02.ops->read_data(&at24c02, 0x00, 1, rbuf);
    uart_debug.ops->printf("1. Byte %s (W=AB R=%02X)\r\n",
                           (!ret && rbuf[0] == 0xAB) ? "PASS" : "FAIL", rbuf[0]);

    /* 2. 整页写读测试 */
    ret = at24c02.ops->write_page(&at24c02, 0x08, wbuf);
    delay_ms(5);
    ret |= at24c02.ops->read_data(&at24c02, 0x08, sizeof(wbuf), rbuf);
    uart_debug.ops->printf("2. Page %s\r\n", (!ret && !memcmp(wbuf,rbuf,sizeof(wbuf)))?"PASS":"FAIL");

    /* 3. 连续读取测试 */
    ret = at24c02.ops->read_data(&at24c02, 0x00, 10, rbuf);
    uart_debug.ops->printf("3. Continuous Read %s: ", ret?"FAIL":"PASS");
    for (uint8_t i = 0; i < 10; i++) 
        uart_debug.ops->printf("%02X ", rbuf[i]);
    uart_debug.ops->printf("\r\n==== All Tests Done ====\r\n");

    while (1)
        delay_s(1);
}
