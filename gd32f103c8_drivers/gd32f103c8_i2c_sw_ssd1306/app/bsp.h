#ifndef BSP_H
#define BSP_H

#include "delay.h"
#include "oled.h"
#include "oled_data.h"

/* 硬件设备声明 */
extern oled_dev_t oled;
// ...

int bsp_init(void);

#endif
