#ifndef BSP_H
#define BSP_H

#include "delay.h"
#include "uart.h"
#include "exti.h"

/* 硬件设备声明 */
extern uart_dev_t debug;
extern exti_dev_t cnt[];
// ...

int bsp_init(void);

#endif
