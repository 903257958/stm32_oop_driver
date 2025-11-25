#ifndef DRV_CAN_H
#define DRV_CAN_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F10X_MD)
#define DRV_CAN_PLATFORM_STM32F1 1
#include "stm32f10x.h"
typedef CAN_TypeDef*	can_periph_t;
typedef GPIO_TypeDef*	gpio_port_t;
typedef uint32_t		gpio_pin_t;
typedef CanTxMsg		can_tx_msg_t;
typedef CanRxMsg		can_rx_msg_t;

#elif defined (GD32F10X_MD)
#define DRV_CAN_PLATFORM_GD32F1 1
#include "gd32f10x.h"
typedef uint32_t					can_periph_t;
typedef uint32_t					gpio_port_t;
typedef uint32_t					gpio_pin_t;
typedef can_trasnmit_message_struct	can_tx_msg_t;
typedef can_receive_message_struct	can_rx_msg_t;

#else
#error drv_can.h: No processor defined!
#endif

#ifndef ETIMEDOUT 
#define ETIMEDOUT	7
#endif

/* CAN 波特率枚举 */
typedef enum {
    CAN_BAUDRATE_125K = 0,
    CAN_BAUDRATE_250K,
    CAN_BAUDRATE_500K,
    CAN_BAUDRATE_1M,
} can_baudrate_t;

/* CAN 过滤器模式 */
typedef enum {
	CAN_FILTER_MODE_16BIT_LIST = 0,
	CAN_FILTER_MODE_16BIT_MASK,
	CAN_FILTER_MODE_32BIT_LIST,
	CAN_FILTER_MODE_32BIT_MASK,
} can_filter_mode_t;

/* CAN 过滤器配置结构体 */
typedef struct {
	uint8_t 		  id;				// 过滤器编号
	can_filter_mode_t mode;				// 过滤器模式
	uint16_t 		  id_high;			// 过滤器ID高16位
	uint16_t 		  id_low;			// 过滤器ID低16位
	uint16_t 		  mask_id_high;		// 过滤器掩码高16位
	uint16_t 		  mask_id_low;		// 过滤器掩码低16位
	uint16_t 		  fifo_assignment;	// 过滤器关联，FIFO0或FIFO1排队
} can_filter_cfg_t;

/* 配置结构体 */
typedef struct {
	can_periph_t 	  can_periph;
	can_baudrate_t 	  baudrate;
	gpio_port_t 	  rx_port;
	gpio_pin_t 		  rx_pin;
	gpio_port_t 	  tx_port;
	gpio_pin_t 		  tx_pin;
	uint8_t 		  mode;			// 模式：环回测试 CAN_LOOPBACK_MODE / 正常模式 CAN_NORMAL_MODE
	can_filter_cfg_t *filter_list;	// 要配置的过滤器列表
	uint8_t 		  filter_num;	// 要配置的过滤器数量
} can_cfg_t;

typedef struct can_dev can_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*send)(can_dev_t *dev, can_tx_msg_t *msg);
	bool (*recv_flag)(can_dev_t *dev, uint8_t fifo_number);
	int (*recv)(can_dev_t *dev, uint8_t fifo_number, can_rx_msg_t *msg);
	int (*deinit)(can_dev_t *dev);
} can_ops_t;

/* 设备结构体 */
struct can_dev {
	void *priv;
	can_cfg_t cfg;
	const can_ops_t *ops;
};

/**
 * @brief   初始化 CAN 设备驱动
 * @param[out] dev can_dev_t 结构体指针
 * @param[in]  cfg can_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_can_init(can_dev_t *dev, const can_cfg_t *cfg);

#endif
