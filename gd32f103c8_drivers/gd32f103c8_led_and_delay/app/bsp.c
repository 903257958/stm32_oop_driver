#include "bsp.h"

/* 硬件设备定义 */
led_dev_t led = {
	.config = { GPIOC, GPIO_PIN_13, GPIO_LEVEL_LOW }
};

/**
 * @brief   初始化 BSP 硬件
 * @return  0 表示成功
 */
int bsp_init(void)
{
    led_init(&led);
    
    return 0;
}
