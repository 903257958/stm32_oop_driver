#ifndef BSP_H
#define BSP_H

#include "delay.h"
#include "uart.h"
#include "sr04.h"

/* 硬件设备声明 */
extern uart_dev_t debug;
extern sr04_dev_t sr04;
// ...

int bsp_init(void);

#endif
