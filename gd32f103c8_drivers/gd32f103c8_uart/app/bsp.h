#ifndef BSP_H
#define BSP_H

#include "delay.h"
#include "uart.h"

/* 硬件设备声明 */
extern uart_dev_t uart0;
extern uart_dev_t uart1;
extern uart_dev_t uart2;
// ...

int bsp_init(void);

#endif
