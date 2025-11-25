#ifndef __UART_H
#define __UART_H

#include "gd32f10x.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>

#define UART0_TX_SIZE   2048    // UART0发送缓冲区长度
#define UART0_RX_SIZE   2048    // UART0接收缓冲区长度
#define UART0_RX_MAX    256     // UART0单次接收最大长度
#define INDEX_BUF_NUM   10      // 索引结构体数组长度

extern uint8_t uart0_tx_data_buf[UART0_TX_SIZE];	// UART0发送数据缓冲区
extern uint8_t uart0_rx_data_buf[UART0_RX_SIZE];	// UART0接收数据缓冲区

/* 串口接收索引结构体 */
typedef struct {
    uint8_t *start;	// 数据段起始地址
    uint8_t *end;	// 数据段结束地址
}UARTRXIndex_t;

/* 串口接收控制块结构体 */
typedef struct {
    uint16_t data_cnt;						// 已接收数据计数
    UARTRXIndex_t index_buf[INDEX_BUF_NUM];	// 索引数组，用于管理多段数据
    UARTRXIndex_t *index_in;				// 写入指针
    UARTRXIndex_t *index_out;				// 读出指针
    UARTRXIndex_t *index_end;				// 索引数组结束位置
}UARTRXControlBlock_t;

extern UARTRXControlBlock_t uart0_rx_control_block;	// UART0接收控制块结构体

/* 函数声明 */
void uart0_init(uint32_t baud);
void uart0_printf(char *format, ...);
void uart0_recv_data(void);

#endif
