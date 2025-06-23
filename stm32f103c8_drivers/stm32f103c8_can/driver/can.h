#ifndef CAN_DRV_H
#define CAN_DRV_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
	typedef CAN_TypeDef*	can_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	typedef CanTxMsg		can_tx_msg_t;
	typedef CanRxMsg		can_rx_msg_t;
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	/* 注意：F4系列驱动未完善，可能存在bug */
    #include "stm32f4xx.h"
	typedef CAN_TypeDef*	can_periph_t;
	typedef GPIO_TypeDef*	gpio_port_t;
	typedef uint32_t		gpio_pin_t;
	typedef CanTxMsg		can_tx_msg_t;
	typedef CanRxMsg		can_rx_msg_t;

#else
    #error can.h: No processor defined!
#endif

#endif

/* CAN过滤器模式 */
typedef enum {
	CAN_FILTER_MODE_16BIT_LIST = 0,
	CAN_FILTER_MODE_16BIT_MASK,
	CAN_FILTER_MODE_32BIT_LIST,
	CAN_FILTER_MODE_32BIT_MASK,
} can_filter_mode_t;

/* CAN过滤器配置结构体 */
typedef struct {
	uint8_t id;					// 过滤器编号
	can_filter_mode_t mode;		// 过滤器模式
	uint16_t id_high;			// 过滤器ID高16位
	uint16_t id_low;			// 过滤器ID低16位
	uint16_t mask_id_high;		// 过滤器掩码高16位
	uint16_t mask_id_low;		// 过滤器掩码低16位
	uint16_t fifo_assignment;	// 过滤器关联，FIFO0或FIFO1排队
} can_filter_config_t;

/* CAN配置结构体 */
typedef struct {
	can_periph_t canx;					// CAN外设
	gpio_port_t rx_port;				// 接收端口
	gpio_pin_t rx_pin;					// 接收引脚
	gpio_port_t tx_port;				// 发送端口
	gpio_pin_t tx_pin;					// 发送引脚
	uint8_t mode;						// 模式：环回测试/正常模式
	can_filter_config_t *filter_list;	// 要配置的过滤器列表
	uint8_t filter_num;          		// 要配置的过滤器数量
} can_config_t;

/* CAN设备结构体 */
typedef struct can_dev {
	can_config_t config;
	bool init_flag;										    // 初始化标志
	int8_t (*send)(struct can_dev *dev, can_tx_msg_t *msg);
	bool (*recv_flag)(struct can_dev *dev, uint8_t fifo_number);
	int8_t (*recv)(struct can_dev *dev, uint8_t fifo_number, can_rx_msg_t *msg);
	int8_t (*deinit)(struct can_dev *dev);					// 去初始化
} can_dev_t;

int8_t can_init(can_dev_t *dev);

#endif
