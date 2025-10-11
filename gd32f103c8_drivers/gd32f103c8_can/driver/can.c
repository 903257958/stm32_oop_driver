#include "can.h"

#ifdef USE_STDPERIPH_DRIVER

/**************************** GD32F1 系列 ****************************/
#if defined (GD32F10X_MD)

#define	can_clock_enable(canx) rcu_periph_clock_enable(RCU_CAN0)

#define can_io_clock_enable(port)									 \
    do {                                           				     \
        if (port == GPIOA)       rcu_periph_clock_enable(RCU_GPIOA); \
        else if (port == GPIOB)  rcu_periph_clock_enable(RCU_GPIOB); \
        else if (port == GPIOC)  rcu_periph_clock_enable(RCU_GPIOC); \
        else if (port == GPIOD)  rcu_periph_clock_enable(RCU_GPIOD); \
        else if (port == GPIOE)  rcu_periph_clock_enable(RCU_GPIOE); \
        else if (port == GPIOF)  rcu_periph_clock_enable(RCU_GPIOF); \
        else if (port == GPIOG)  rcu_periph_clock_enable(RCU_GPIOG); \
    } while (0)

#define can_config_io_af_pp(port, pin) \
    gpio_init(port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, pin)

#define can_config_io_in_pu(port, pin) \
    gpio_init(port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pin)

#endif  /* MCU SERIES SELECTION */

#endif  /* USE_STDPERIPH_DRIVER */

/* 函数声明 */
static int can_send(can_dev_t *dev, can_tx_msg_t *msg);
static bool can_recv_flag(can_dev_t *dev, uint8_t fifo);
static int can_recv(can_dev_t *dev, uint8_t fifo, can_rx_msg_t *msg);
static int can_drv_deinit(can_dev_t *dev);

/**
 * @brief   初始化 CAN
 * @param[in,out] dev can_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int can_drv_init(can_dev_t *dev)
{
	if (!dev)
		return -1;

	uint8_t i;
	
	/* 配置时钟与 GPIO */	
	can_clock_enable(dev->config.canx);
	can_io_clock_enable(dev->config.rx_port);
	can_io_clock_enable(dev->config.tx_port);

	can_config_io_af_pp(dev->config.tx_port, dev->config.tx_pin);	// 发送引脚初始化为复用推挽输出
	can_config_io_in_pu(dev->config.rx_port, dev->config.rx_pin);	// 接收引脚初始化为上拉输入

	/* CAN外设初始化，Baud = 54MHz / (BRP+1) / (1 + (TS1+1) + (TS2+1)) = 54M / 36 / (1 + 2 + 3) = 250KHz */
	can_parameter_struct can_parameter_init;
	can_parameter_init.working_mode = dev->config.mode;		// 模式：环回测试/正常模式
	can_parameter_init.prescaler = 36;						// 分频系数，BRP + 1 的值
	can_parameter_init.time_segment_1 = CAN_BT_BS1_2TQ;		// TS1 + 1 的值
	can_parameter_init.time_segment_2 = CAN_BT_BS2_3TQ;		// TS2 + 1 的值
	can_parameter_init.resync_jump_width = CAN_BT_SJW_1TQ;	// 再同步补偿宽度，仅用于再同步，与波特率的计算无关
	can_parameter_init.rec_fifo_overwrite = DISABLE;		// FIFO锁定，置1，FIFO溢出时新收到的报文被丢弃；置0，FIFO溢出时最后收到的报文被新报文覆盖
	can_parameter_init.time_triggered = DISABLE;			// 置1，开启时间触发通信功能；置0，关闭时间触发通信功能	
	can_parameter_init.trans_fifo_order = DISABLE;			// 发送邮箱优先级，置1，先请求先发送；置0，标识符值小先发送
	can_parameter_init.auto_bus_off_recovery = DISABLE;		// 置1，开启离线自动恢复；置0，关闭离线自动恢复
	can_parameter_init.auto_retrans = DISABLE;				// 置1，不自动重传；置0，自动重传
	can_parameter_init.auto_wake_up = DISABLE;				// 置1，自动唤醒；置0，手动唤醒
	can_init(dev->config.canx, &can_parameter_init);

	/* 配置过滤器 */
	for (i = 0; i < dev->config.filter_num; i++) {
		const can_filter_config_t *filter = &dev->config.filter_list[i];

		can_filter_parameter_struct can_filter_parameter_init;
		can_filter_parameter_init.filter_number = filter->id;

		/* 过滤器模式 */
		if (filter->mode == CAN_FILTER_MODE_16BIT_LIST) {
			can_filter_parameter_init.filter_bits = CAN_FILTERBITS_16BIT;	// 过滤器位宽
			can_filter_parameter_init.filter_mode = CAN_FILTERMODE_LIST;	// 过滤器模式
		} else if (filter->mode == CAN_FILTER_MODE_16BIT_MASK) {
			can_filter_parameter_init.filter_bits = CAN_FILTERBITS_16BIT;	// 过滤器位宽
			can_filter_parameter_init.filter_mode = CAN_FILTERMODE_MASK;	// 过滤器模式
		} else if (filter->mode == CAN_FILTER_MODE_32BIT_LIST) {
			can_filter_parameter_init.filter_bits = CAN_FILTERBITS_32BIT;	// 过滤器位宽
			can_filter_parameter_init.filter_mode = CAN_FILTERMODE_LIST;	// 过滤器模式
		} else if (filter->mode == CAN_FILTER_MODE_32BIT_MASK) {
			can_filter_parameter_init.filter_bits = CAN_FILTERBITS_32BIT;	// 过滤器位宽
			can_filter_parameter_init.filter_mode = CAN_FILTERMODE_MASK;	// 过滤器模式
		}

		/*
		 * 16位列表模式：4个参数分别存入1组ID
		 * 16位屏蔽模式：IdHigh存入第1组ID，MaskIdHigh存入对应屏蔽位；IdLow存入第2组ID，MaskIdLow存入对应屏蔽位
		 * 32位列表模式：IdHigh与IdLow组合存入第1组ID，MaskIdHigh与MaskIdLow组合存入第2组ID
		 * 32位屏蔽模式：IdHigh与IdLow组合存入ID，MaskIdHigh与MaskIdLow组合存入对应屏蔽位
		 * 32Bit：11位STID、18位EXID、1位IDE、1位RTR、1位0
		 * 16Bit：11位STID、1位RTR、1位IDE、3位0
		 */
		can_filter_parameter_init.filter_list_high = filter->id_high;			// 过滤器ID高16位
		can_filter_parameter_init.filter_list_low = filter->id_low;				// 过滤器ID低16位
		can_filter_parameter_init.filter_mask_high = filter->mask_id_high;		// 过滤器掩码高16位
		can_filter_parameter_init.filter_mask_low = filter->mask_id_low;		// 过滤器掩码低16位
		can_filter_parameter_init.filter_fifo_number = filter->fifo_assignment;	// 过滤器关联，FIFO0或FIFO1排队
		can_filter_parameter_init.filter_enable = ENABLE;						// 激活过滤器
		can_filter_init(&can_filter_parameter_init);
	}

	/* 函数指针赋值 */
	dev->send = can_send;
	dev->recv_flag = can_recv_flag;
	dev->recv = can_recv;
	dev->deinit = can_drv_deinit;
	
	dev->init_flag = true;
	return 0;
}

/**
 * @brief   CAN 发送
 * @param[in] dev can_dev_t 结构体指针
 * @param[in] msg 发送数据结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int can_send(can_dev_t *dev, can_tx_msg_t *msg)
{
	if (!dev || !dev->init_flag)
		return -1;

	uint32_t timeout = 0;
	uint8_t mailbox_number;	// 写入的邮箱号

	mailbox_number = can_message_transmit(dev->config.canx, msg);
	
	/* 检查邮箱状态，等待发送成功 */
	while (can_transmit_states(dev->config.canx, mailbox_number) != CAN_TRANSMIT_OK) {
		timeout++;
		if (timeout > 100000)
			return -2;
	}

	return 0;
}

/**
 * @brief   CAN 接收标志位，判断接收 FIFO 中是否有报文
 * @param[in] dev  can_dev_t 结构体指针
 * @param[in] fifo 指定哪个 FIFO： CAN_FIFO0 / CAN_FIFO1
 * @return	true 表示有报文, false 表示无报文
 */
static bool can_recv_flag(can_dev_t *dev, uint8_t fifo)
{
	if (can_receive_message_length_get(dev->config.canx, fifo) > 0)	// 检查FIFO的队列长度
		return true;
	else
		return false;
}

/**
 * @brief   CAN 接收
 * @param[in]  dev  can_dev_t 结构体指针
 * @param[in]  fifo 指定哪个 FIFO
 * @param[out] msg  接收数据结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int can_recv(can_dev_t *dev, uint8_t fifo, can_rx_msg_t *msg)
{
	if (!dev || !dev->init_flag)
		return -1;

	can_message_receive(dev->config.canx, fifo, msg);
	
	return 0;
}

/**
 * @brief   去初始化 CAN
 * @param[in,out] dev can_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int can_drv_deinit(can_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;
	
	return 0;
}
