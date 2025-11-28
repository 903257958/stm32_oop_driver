#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_i2c_soft.h"
#include "drv_aht21.h"
#include <string.h>

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

static i2c_soft_dev_t i2c_soft;
static const i2c_soft_cfg_t i2c_soft_cfg = {
    .scl_port     = GPIOB, 
    .scl_pin      = GPIO_Pin_6, 
    .sda_port     = GPIOB, 
    .sda_pin      = GPIO_Pin_7,
    .delay_us     = delay_us, 
    .bit_delay_us = 1
};

int i2c_soft_write_regs_adapter(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data)
{
    return i2c_soft.ops->write_regs(&i2c_soft, dev_addr, reg_addr, num, data);
}

int i2c_soft_read_reg_adapter(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return i2c_soft.ops->read_reg(&i2c_soft, dev_addr, reg_addr, data);
}

int i2c_soft_read_regs_adapter(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data)
{
    return i2c_soft.ops->read_regs(&i2c_soft, dev_addr, reg_addr, num, data);
}

static aht21_i2c_ops_t aht21_i2c_ops = {
    .write_regs = i2c_soft_write_regs_adapter, 
    .read_reg   = i2c_soft_read_reg_adapter,
    .read_regs  = i2c_soft_read_regs_adapter
};

static aht21_dev_t aht21;
static const aht21_cfg_t aht21_cfg = {
    .delay_ms = delay_ms,
    .i2c_ops  = &aht21_i2c_ops, 
};

int main(void)
{
    aht21_data_t data;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_i2c_soft_init(&i2c_soft, &i2c_soft_cfg);
    drv_aht21_init(&aht21, &aht21_cfg);

    while (1) {
        aht21.ops->get_data(&aht21, &data);
        uart_debug.ops->printf(&uart_debug, "Temperature: %f\r\n", data.temperature);
        uart_debug.ops->printf(&uart_debug, "Humidity: %f\r\n\r\n", data.humidity);
        delay_ms(500);
    }
}
