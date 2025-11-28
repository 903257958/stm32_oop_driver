#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_timer.h"
#include "drv_key.h"
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

static timer_dev_t timer_key;
static const timer_cfg_t timer_key_cfg = {
    .timer_periph = TIM2,
    .psc = 72 - 1,
    .arr = 1000 - 1,
    .pre_priority = 0,
    .sub_priority = 0,
    .use_irq = true
};

static key_dev_t key[2];
static const key_cfg_t key_cfg[] = {
    {
        .port              = GPIOB,
        .pin               = GPIO_Pin_12,
        .press_level       = GPIO_LEVEL_LOW,
        .debounce_ms       = 20,
        .double_timeout_ms = 100,
        .long_timeout_ms   = 1000,
        .repeat_timeout_ms = 100,
        .enable_double     = true,
        .enable_long       = true,
        .enable_repeat     = true
    },
    {
        .port              = GPIOA,
        .pin               = GPIO_Pin_8,
        .press_level       = GPIO_LEVEL_LOW,
        .debounce_ms       = 20,
        .double_timeout_ms = 100,
        .long_timeout_ms   = 1000,
        .repeat_timeout_ms = 100,
        .enable_double     = false,
        .enable_long       = true,
        .enable_repeat     = false
    }
};

static void key_down_callback(void *param)
{
    uart_debug.ops->printf(&uart_debug, "%s down\r\n", (char *)param);
}

static void key_up_callback(void *param)
{
    uart_debug.ops->printf(&uart_debug, "%s up\r\n", (char *)param);
}

static void key_click_callback(void *param)
{
    uart_debug.ops->printf(&uart_debug, "%s clicked\r\n", (char *)param);
}

static void key_double_callback(void *param)
{
    uart_debug.ops->printf(&uart_debug, "%s double clicked\r\n", (char *)param);
}

static void key_long_callback(void *param)
{
    uart_debug.ops->printf(&uart_debug, "%s long pressed\r\n", (char *)param);
}

static void key_repeat_callback(void *param)
{
    uart_debug.ops->printf(&uart_debug, "%s repeat\r\n", (char *)param);
}

int main(void)
{
    uint8_t i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_timer_init(&timer_key, &timer_key_cfg);
    for (i = 0; i < sizeof(key_cfg) / sizeof(key_cfg[0]); i++)
        drv_key_init(&key[i], &key_cfg[i]);

    key[0].ops->register_callback(&key[0], KEY_EVENT_DOWN,   key_down_callback,   "Key0");
    key[0].ops->register_callback(&key[0], KEY_EVENT_UP,     key_up_callback,     "Key0");
    key[0].ops->register_callback(&key[0], KEY_EVENT_CLICK,  key_click_callback,  "Key0");
    key[0].ops->register_callback(&key[0], KEY_EVENT_DOUBLE, key_double_callback, "Key0");
    key[0].ops->register_callback(&key[0], KEY_EVENT_LONG,   key_long_callback,   "Key0");
    key[0].ops->register_callback(&key[0], KEY_EVENT_REPEAT, key_repeat_callback, "Key0");

    key[1].ops->register_callback(&key[1], KEY_EVENT_DOWN,   key_down_callback,   "Key1");
    key[1].ops->register_callback(&key[1], KEY_EVENT_UP,     key_up_callback,     "Key1");
    key[1].ops->register_callback(&key[1], KEY_EVENT_CLICK,  key_click_callback,  "Key1");
    key[1].ops->register_callback(&key[1], KEY_EVENT_DOUBLE, key_double_callback, "Key1");
    key[1].ops->register_callback(&key[1], KEY_EVENT_LONG,   key_long_callback,   "Key1");
    key[1].ops->register_callback(&key[1], KEY_EVENT_REPEAT, key_repeat_callback, "Key1");

#if 1
    /* 使用定时器中断，必须 1ms 调用一次按键 Tick */
    timer_key.ops->register_irq_callback(&timer_key, drv_key_tick, NULL);

    while (1) {
        for (i = 0; i < sizeof(key) / sizeof(key[0]); i++) {
            key[i].ops->process_event(&key[i]);
        }
        delay_ms(1);
	}
#else
    /* 使用延时轮询，必须 1ms 调用一次按键 Tick */
    while (1) {
        drv_key_tick(NULL);
        for (i = 0; i < sizeof(key) / sizeof(key[0]); i++)
            key[i].ops->process_event(&key[i]);
        delay_ms(1);
	}
#endif
}
