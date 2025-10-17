#ifndef BSP_H
#define BSP_H

#include "delay.h"
#include "uart.h"
#include "timer.h"

/* 硬件设备声明 */
extern uart_dev_t debug;
extern timer_dev_t timer1;
extern timer_dev_t timer2;
// ...

int bsp_init(void);

#endif
