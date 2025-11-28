#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_exti.h"

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

static exti_dev_t exti[2];
static const exti_cfg_t exti_cfg[2] = {
    [0] = {
        .port         = GPIOC, 
        .pin          = GPIO_Pin_1, 
        .trigger      = EXTI_Trigger_Falling, 
        .pre_priority = 0, 
        .sub_priority = 0
    },
    [1] = {
        .port         = GPIOC, 
        .pin          = GPIO_Pin_2, 
        .trigger      = EXTI_Trigger_Falling, 
        .pre_priority = 0, 
        .sub_priority = 0
    }
};

static void cnt_callback(void *param)
{
    static int cnt = 0;
    const char *name = (const char *)param;

    uart_debug.ops->printf(&uart_debug, "%s: cnt = %d\r\n", name, ++cnt);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    for (int i = 0; i < sizeof(exti) / sizeof(exti[0]); i++)
        drv_exti_init(&exti[i], &exti_cfg[i]);

    exti[0].falling_irq_callback       = cnt_callback;
    exti[0].falling_irq_callback_param = "EXTI0";
    exti[1].falling_irq_callback       = cnt_callback;
    exti[1].falling_irq_callback_param = "EXTI1";
    
	while (1) {
        delay_s(1);
    }
}
