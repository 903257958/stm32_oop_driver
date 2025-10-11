#ifndef BSP_H
#define BSP_H

#include "delay.h"
#include "uart.h"
#include "can.h"

/* 硬件设备声明 */
extern uart_dev_t debug;
extern can_dev_t can0;
// ...

int bsp_init(void);

#endif
