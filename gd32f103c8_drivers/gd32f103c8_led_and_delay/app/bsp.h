#ifndef BSP_H
#define BSP_H

#include "delay.h"
#include "led.h"

/* 硬件设备声明 */
extern led_dev_t led;
// ...

int bsp_init(void);

#endif
