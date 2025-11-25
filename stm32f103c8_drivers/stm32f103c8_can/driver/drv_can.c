#include "drv_can.h"
#include <errno.h>
#include <stddef.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 CAN 外设时钟
 * @param[in] can_periph CAN 外设
 */
static void can_hw_can_clock_enable(can_periph_t can_periph)
{
#if DRV_CAN_PLATFORM_STM32F1
    switch ((uint32_t)can_periph) {
	case (uint32_t)CAN1: RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); break;
	default: break;
    }
#elif DRV_CAN_PLATFORM_GD32F1
	switch ((uint32_t)can_periph) {
	case (uint32_t)CAN0: rcu_periph_clock_enable(RCU_CAN0); break;
	default: break;
    }
#endif
}

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void can_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_CAN_PLATFORM_STM32F1
    switch ((uint32_t)port) {
    case (uint32_t)GPIOA: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); break;
    case (uint32_t)GPIOB: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); break;
    case (uint32_t)GPIOC: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); break;
    case (uint32_t)GPIOD: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); break;
    case (uint32_t)GPIOE: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); break;
    case (uint32_t)GPIOF: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); break;
    case (uint32_t)GPIOG: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_CAN_PLATFORM_GD32F1
	switch ((uint32_t)port) {
    case (uint32_t)GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
    case (uint32_t)GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
    case (uint32_t)GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
    case (uint32_t)GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
    case (uint32_t)GPIOE: rcu_periph_clock_enable(RCU_GPIOE); break;
    case (uint32_t)GPIOF: rcu_periph_clock_enable(RCU_GPIOF); break;
    case (uint32_t)GPIOG: rcu_periph_clock_enable(RCU_GPIOG); break;
	default: break;
    }
#endif
}

/**
 * @brief	初始化 GPIO 为 CAN 功能
 * @param[in] cfg can_cfg_t 结构体指针
 */
static void can_hw_gpio_init(const can_cfg_t *cfg)
{
#if DRV_CAN_PLATFORM_STM32F1
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = cfg->tx_pin;
    GPIO_Init(cfg->tx_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin  = cfg->rx_pin;
    GPIO_Init(cfg->rx_port, &GPIO_InitStructure);

#elif DRV_CAN_PLATFORM_GD32F1
	gpio_init(cfg->tx_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, cfg->tx_pin);
	gpio_init(cfg->rx_port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, cfg->rx_pin);
#endif
}

/**
 * @brief	初始化 CAN 外设
 * @param[in] cfg can_cfg_t 结构体指针
 */
static void can_hw_can_init(const can_cfg_t *cfg)
{
#if DRV_CAN_PLATFORM_STM32F1
	/* 
	 * CAN外设初始化
	 * 波特率计算示例：
	 * Baud = 36MHz / (BRP+1) / (1 + (TS1+1) + (TS2+1)) = 36M / 24 / (1 + 2 + 3) = 250KHz 
	 */
	CAN_InitTypeDef CAN_InitStructure;
	CAN_InitStructure.CAN_Mode = cfg->mode;		// 模式：环回测试/正常模式
	if (cfg->baudrate == CAN_BAUDRATE_125K)
		CAN_InitStructure.CAN_Prescaler = 48;	// 分频系数，BRP + 1 的值
	else if (cfg->baudrate == CAN_BAUDRATE_250K)
		CAN_InitStructure.CAN_Prescaler = 24;	// 分频系数，BRP + 1 的值
	else if (cfg->baudrate == CAN_BAUDRATE_500K)
		CAN_InitStructure.CAN_Prescaler = 12;	// 分频系数，BRP + 1 的值
	else if (cfg->baudrate == CAN_BAUDRATE_1M)
		CAN_InitStructure.CAN_Prescaler = 6;	// 分频系数，BRP + 1 的值
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;	// TS1 + 1 的值
	CAN_InitStructure.CAN_BS2 = CAN_BS1_3tq;	// TS2 + 1 的值
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;	// 再同步补偿宽度，仅用于再同步，与波特率的计算无关
	CAN_InitStructure.CAN_NART = DISABLE;		// 置1，不自动重传；置0，自动重传
	CAN_InitStructure.CAN_TXFP = DISABLE;		// 发送邮箱优先级，置1，先请求先发送；置0，标识符值小先发送
	CAN_InitStructure.CAN_RFLM = DISABLE;		// FIFO锁定，置1，FIFO溢出时新收到的报文被丢弃；置0，FIFO溢出时最后收到的报文被新报文覆盖
	CAN_InitStructure.CAN_AWUM = DISABLE;		// 置1，自动唤醒；置0，手动唤醒
	CAN_InitStructure.CAN_TTCM = DISABLE;		// 置1，开启时间触发通信功能；置0，关闭时间触发通信功能
	CAN_InitStructure.CAN_ABOM = DISABLE;		// 置1，开启离线自动恢复；置0，关闭离线自动恢复
	CAN_Init(cfg->can_periph, &CAN_InitStructure);

	/* 配置过滤器 */
	for (uint8_t i = 0; i < cfg->filter_num; i++) {
		const can_filter_cfg_t *filter = &cfg->filter_list[i];
		
		CAN_FilterInitTypeDef CAN_FilterInitStructure;
		CAN_FilterInitStructure.CAN_FilterNumber = filter->id;
		
		/* 过滤器模式 */
		if (filter->mode == CAN_FILTER_MODE_16BIT_LIST) {
			CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;	// 过滤器位宽
			CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;		// 过滤器模式
		} else if (filter->mode == CAN_FILTER_MODE_16BIT_MASK) {
			CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;	// 过滤器位宽
			CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		// 过滤器模式
		} else if (filter->mode == CAN_FILTER_MODE_32BIT_LIST) {
			CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// 过滤器位宽
			CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;		// 过滤器模式
		} else if (filter->mode == CAN_FILTER_MODE_32BIT_MASK) {
			CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// 过滤器位宽
			CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		// 过滤器模式
		}

		/*
		 * 16位列表模式：4个参数分别存入1组ID
		 * 16位屏蔽模式：IdHigh存入第1组ID，MaskIdHigh存入对应屏蔽位；IdLow存入第2组ID，MaskIdLow存入对应屏蔽位
		 * 32位列表模式：IdHigh与IdLow组合存入第1组ID，MaskIdHigh与MaskIdLow组合存入第2组ID
		 * 32位屏蔽模式：IdHigh与IdLow组合存入ID，MaskIdHigh与MaskIdLow组合存入对应屏蔽位
		 * 32Bit：11位STID、18位EXID、1位IDE、1位RTR、1位0
		 * 16Bit：11位STID、1位RTR、1位IDE、3位0
		 */
		CAN_FilterInitStructure.CAN_FilterIdHigh = filter->id_high;					// 过滤器ID高16位
		CAN_FilterInitStructure.CAN_FilterIdLow = filter->id_low;					// 过滤器ID低16位
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = filter->mask_id_high;		// 过滤器掩码高16位
		CAN_FilterInitStructure.CAN_FilterMaskIdLow = filter->mask_id_low;			// 过滤器掩码低16位
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = filter->fifo_assignment;	// 过滤器关联，FIFO0或FIFO1排队
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;						// 激活过滤器

		CAN_FilterInit(&CAN_FilterInitStructure);
	}

#elif DRV_CAN_PLATFORM_GD32F1
	/* 
	 * CAN外设初始化
	 * 波特率计算示例：
	 * Baud = 54MHz / (BRP+1) / (1 + (TS1+1) + (TS2+1)) = 54M / 36 / (1 + 2 + 3) = 250KHz 
	 */
	can_parameter_struct can_parameter_init;
	can_parameter_init.working_mode = cfg->mode;			// 模式：环回测试/正常模式
	if (cfg->baudrate == CAN_BAUDRATE_125K)
		can_parameter_init.prescaler = 72;					// 分频系数，BRP + 1 的值
	else if (cfg->baudrate == CAN_BAUDRATE_250K)
		can_parameter_init.prescaler = 36;					// 分频系数，BRP + 1 的值
	else if (cfg->baudrate == CAN_BAUDRATE_500K)
		can_parameter_init.prescaler = 18;					// 分频系数，BRP + 1 的值
	else if (cfg->baudrate == CAN_BAUDRATE_1M)
		can_parameter_init.prescaler = 9;					// 分频系数，BRP + 1 的值
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
	can_init(cfg->can_periph, &can_parameter_init);

	/* 配置过滤器 */
	for (uint8_t i = 0; i < cfg->filter_num; i++) {
		const can_filter_cfg_t *filter = &cfg->filter_list[i];

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
#endif
}

/**
 * @brief   发送 CAN 消息
 * @param[in] can_periph CAN 外设
 * @param[in] msg 		 发送数据结构体指针
 * @return	
 */
static uint8_t can_hw_transmit(can_periph_t can_periph, can_tx_msg_t *msg)
{
#if DRV_CAN_PLATFORM_STM32F1
	return CAN_Transmit(can_periph, msg);
#elif DRV_CAN_PLATFORM_GD32F1
	return can_message_transmit(can_periph, msg);
#endif
}

/**
 * @brief	检查 CAN 发送消息状态
 * @param[in] can_periph 	 CAN 外设
 * @param[in] mailbox_number 邮箱号
 * @return	1 表示发送成功，其他表示未发送成功
 */
static uint8_t can_hw_get_transmit_status(can_periph_t can_periph, uint8_t mailbox_number)
{
#if DRV_CAN_PLATFORM_STM32F1
	return CAN_TransmitStatus(can_periph, mailbox_number);
#elif DRV_CAN_PLATFORM_GD32F1
	return can_transmit_states(can_periph, mailbox_number);
#endif
}

/**
 * @brief	CAN 获取待处理消息的数量
 * @param[in] can_periph CAN 外设
 * @param[in] fifo 		 FIFO
 * @return	待处理消息的数量
 */
static uint8_t can_hw_get_msg_pending(can_periph_t can_periph, uint8_t fifo)
{
#if DRV_CAN_PLATFORM_STM32F1
	return CAN_MessagePending(can_periph, fifo);
#elif DRV_CAN_PLATFORM_GD32F1
	return can_receive_message_length_get(can_periph, fifo);
#endif
}

/**
 * @brief   接收 CAN 消息
 * @param[in]  can_periph CAN 外设
 * @param[in]  fifo 	  FIFO
 * @param[out] msg 		  接收数据的结构体指针
 */
static void can_hw_receive(can_periph_t can_periph, uint8_t fifo, can_rx_msg_t *msg)
{
#if DRV_CAN_PLATFORM_STM32F1
	CAN_Receive(can_periph, fifo, msg);
#elif DRV_CAN_PLATFORM_GD32F1
	can_message_receive(can_periph, fifo, msg);
#endif
}

/**
 * @brief   初始化 CAN 硬件
 * @param[in] cfg can_cfg_t 结构体指针
 */
static void can_hw_init(const can_cfg_t *cfg)
{
	can_hw_can_clock_enable(cfg->can_periph);
	can_hw_gpio_clock_enable(cfg->tx_port);
	can_hw_gpio_clock_enable(cfg->rx_port);

	can_hw_gpio_init(cfg);
	can_hw_can_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int can_send_impl(can_dev_t *dev, can_tx_msg_t *msg);
static bool can_recv_impl_flag_impl(can_dev_t *dev, uint8_t fifo);
static int can_recv_impl(can_dev_t *dev, uint8_t fifo, can_rx_msg_t *msg);
static int can_deinit_impl(can_dev_t *dev);

/* 操作接口表 */
static const can_ops_t can_ops = {
	.send      = can_send_impl,
	.recv_flag = can_recv_impl_flag_impl,
	.recv      = can_recv_impl,
	.deinit	   = can_deinit_impl
};

/**
 * @brief   初始化 CAN 设备驱动
 * @param[out] dev can_dev_t 结构体指针
 * @param[in]  cfg can_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int drv_can_init(can_dev_t *dev, const can_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;
	
	dev->cfg = *cfg;
	dev->ops = &can_ops;

	can_hw_init(cfg);
	return 0;
}

/**
 * @brief   CAN 发送
 * @param[in] dev can_dev_t 结构体指针
 * @param[in] msg 发送数据结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int can_send_impl(can_dev_t *dev, can_tx_msg_t *msg)
{
	if (!dev)
		return -EINVAL;

	uint32_t timeout = 0;
	uint8_t mailbox_number;	// 写入的邮箱号

	mailbox_number = can_hw_transmit(dev->cfg.can_periph, msg);
	
	/* 检查邮箱状态，等待发送成功 */
	while (can_hw_get_transmit_status(dev->cfg.can_periph, mailbox_number) != 1) {
		timeout++;
		if (timeout > 100000)
			return -ETIMEDOUT;
	}
	return 0;
}

/**
 * @brief   CAN 接收标志位，判断接收 FIFO 中是否有报文
 * @param[in] dev  can_dev_t 结构体指针
 * @param[in] fifo 指定哪个 FIFO： CAN_FIFO0 / CAN_FIFO1
 * @return	true 表示有报文, false 表示无报文
 */
static bool can_recv_impl_flag_impl(can_dev_t *dev, uint8_t fifo)
{
	if (can_hw_get_msg_pending(dev->cfg.can_periph, fifo) > 0)	// 检查FIFO的队列长度
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
static int can_recv_impl(can_dev_t *dev, uint8_t fifo, can_rx_msg_t *msg)
{
	if (!dev)
		return -EINVAL;

	can_hw_receive(dev->cfg.can_periph, fifo, msg);
	return 0;
}

/**
 * @brief   去初始化 CAN
 * @param[in,out] dev can_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int can_deinit_impl(can_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
