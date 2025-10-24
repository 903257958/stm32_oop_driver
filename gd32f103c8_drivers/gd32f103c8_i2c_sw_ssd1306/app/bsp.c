#include "bsp.h"

/* 硬件设备定义 */
oled_dev_t oled = {
    .config = { GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7 }
};

/**
 * @brief   初始化 BSP 硬件
 * @return  0 表示成功
 */
int bsp_init(void)
{
    oled_drv_init(&oled);
    
    return 0;
}
