#include "can.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define	__can_clock_enable(canx)	if(canx == CAN1)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);} \

#define	__can_io_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
										else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
										else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
										else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
										else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
										else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
										else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
									}

#define	__can_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__can_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__can_clock_enable(canx)	{	if(canx == CAN1)		{;} \
										else if(canx == CAN2)	{;} \
									}

#define	__can_io_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
										else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
										else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
										else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
										else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
										else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
										else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
									}

// #define	__can_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
// 												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
// 												GPIO_InitStructure.GPIO_Pin = pin; \
// 												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
// 												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
// 												GPIO_Init(port, &GPIO_InitStructure); \
// 											}

// #define	__can_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
// 												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
// 												GPIO_InitStructure.GPIO_Pin = pin; \
// 												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
// 												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
// 												GPIO_Init(port, &GPIO_InitStructure); \
}

#endif

#endif

/* 函数声明 */
static int8_t __can_send(can_dev_t *dev, can_tx_msg_t *msg);
static bool __can_recv_flag(can_dev_t *dev, uint8_t fifo_number);
static int8_t __can_recv(can_dev_t *dev, uint8_t fifo_number, can_rx_msg_t *msg);
static int8_t __can_deinit(can_dev_t *dev);

/******************************************************************************
 * @brief	初始化CAN
 * @param	dev	:  can_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t can_init(can_dev_t *dev)
{
	if (!dev)
		return -1;

	uint8_t i;
	
	/* 开启时钟 */	
	__can_clock_enable(dev->config.canx);
	__can_io_clock_enable(dev->config.rx_port);
	__can_io_clock_enable(dev->config.tx_port);

	/* 配置GPIO */
	__can_config_io_af_pp(dev->config.tx_port, dev->config.tx_pin);	// 发送引脚初始化为复用推挽输出
	__can_config_io_in_pu(dev->config.rx_port, dev->config.rx_pin);	// 接收引脚初始化为上拉输入
	
	/* CAN外设初始化，Baud = 36MHz / (BRP+1) / (1 + (TS1+1) + (TS2+1)) = 36M / 48 / (1 + 2 + 3) = 125KHz */
	CAN_InitTypeDef CAN_InitStructure;
	CAN_InitStructure.CAN_Mode = dev->config.mode;	// 模式：环回测试/正常模式
	CAN_InitStructure.CAN_Prescaler = 48;			// 分频系数，BRP + 1 的值
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;		// TS1 + 1 的值
	CAN_InitStructure.CAN_BS2 = CAN_BS1_3tq;		// TS2 + 1 的值
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;		// 再同步补偿宽度，仅用于再同步，与波特率的计算无关
	CAN_InitStructure.CAN_NART = DISABLE;			// 置1，不自动重传；置0，自动重传
	CAN_InitStructure.CAN_TXFP = DISABLE;			// 发送邮箱优先级，置1，先请求先发送；置0，标识符值小先发送
	CAN_InitStructure.CAN_RFLM = DISABLE;			// FIFO锁定，置1，FIFO溢出时新收到的报文被丢弃；置0，FIFO溢出时最后收到的报文被新报文覆盖
	CAN_InitStructure.CAN_AWUM = DISABLE;			// 置1，自动唤醒；置0，手动唤醒
	CAN_InitStructure.CAN_TTCM = DISABLE;			// 置1，开启时间触发通信功能；置0，关闭时间触发通信功能
	CAN_InitStructure.CAN_ABOM = DISABLE;			// 置1，开启离线自动恢复；置0，关闭离线自动恢复
	CAN_Init(dev->config.canx, &CAN_InitStructure);

	/* 配置过滤器 */
	for (i = 0; i < dev->config.filter_num; i++)
	{
		const can_filter_config_t *filter = &dev->config.filter_list[i];
		
		CAN_FilterInitTypeDef CAN_FilterInitStructure;
		CAN_FilterInitStructure.CAN_FilterNumber = filter->id;
		
		/* 过滤器模式 */
		if (filter->mode == CAN_FILTER_MODE_16BIT_LIST)
		{
			CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;	// 过滤器位宽
			CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;		// 过滤器模式
		}
		else if (filter->mode == CAN_FILTER_MODE_16BIT_MASK)
		{
			CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;	// 过滤器位宽
			CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		// 过滤器模式
		}
		else if (filter->mode == CAN_FILTER_MODE_32BIT_LIST)
		{
			CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// 过滤器位宽
			CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;		// 过滤器模式
		}
		else if (filter->mode == CAN_FILTER_MODE_32BIT_MASK)
		{
			CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// 过滤器位宽
			CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		// 过滤器模式
		}

		/* 16位列表模式：4个参数分别存入1组ID */
		/* 16位屏蔽模式：IdHigh存入第1组ID，MaskIdHigh存入对应屏蔽位；IdLow存入第2组ID，MaskIdLow存入对应屏蔽位 */
		/* 32位列表模式：IdHigh与IdLow组合存入第1组ID，MaskIdHigh与MaskIdLow组合存入第2组ID */
		/* 32位屏蔽模式：IdHigh与IdLow组合存入ID，MaskIdHigh与MaskIdLow组合存入对应屏蔽位 */
		/* 32Bit：11位STID、18位EXID、1位IDE、1位RTR、1位0 */
		/* 16Bit：11位STID、1位RTR、1位IDE、3位0 */
		CAN_FilterInitStructure.CAN_FilterIdHigh = filter->id_high;					// 过滤器ID高16位
		CAN_FilterInitStructure.CAN_FilterIdLow = filter->id_low;					// 过滤器ID低16位
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = filter->mask_id_high;		// 过滤器掩码高16位
		CAN_FilterInitStructure.CAN_FilterMaskIdLow = filter->mask_id_low;			// 过滤器掩码低16位
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = filter->fifo_assignment;	// 过滤器关联，FIFO0或FIFO1排队
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;						// 激活过滤器

		CAN_FilterInit(&CAN_FilterInitStructure);
	}

	/* 函数指针赋值 */
	dev->send = __can_send;
	dev->recv_flag = __can_recv_flag;
	dev->recv = __can_recv;
	dev->deinit = __can_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	CAN发送
 * @param	dev	:  can_dev_t 结构体指针
 * @param	msg	:  发送数据结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __can_send(can_dev_t *dev, can_tx_msg_t *msg)
{
	if (!dev || !dev->init_flag)
		return -1;

	uint32_t timeout = 0;
	uint8_t transmit_mailbox;	// 写入的邮箱号

	transmit_mailbox = CAN_Transmit(dev->config.canx, msg);

	while (CAN_TransmitStatus(dev->config.canx, transmit_mailbox) != CAN_TxStatus_Ok)	// 检查邮箱状态，等待发送成功
	{
		timeout++;
		if (timeout > 100000)
		{
			return -2;
		}
	}

	return 0;
}

/******************************************************************************
 * @brief	CAN接收标志位，判断接收FIFO中是否有报文
 * @param	dev			:	can_dev_t 结构体指针
 * @param	fifo_number	:	指定哪个FIFO
 * @return	true, 表示有报文, false，表示无报文
 ******************************************************************************/
static bool __can_recv_flag(can_dev_t *dev, uint8_t fifo_number)
{
	if (fifo_number == 0)
	{
		if (CAN_MessagePending(dev->config.canx, CAN_FIFO0) > 0)	// 检查FIFO的队列长度
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else if (fifo_number == 1)
	{
		if (CAN_MessagePending(dev->config.canx, CAN_FIFO1) > 0)	// 检查FIFO的队列长度
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

/******************************************************************************
 * @brief	CAN接收
 * @param	dev			:	can_dev_t 结构体指针
 * @param	fifo_number	:	指定哪个FIFO
 * @param	msg			:	接收数据结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __can_recv(can_dev_t *dev, uint8_t fifo_number, can_rx_msg_t *msg)
{
	if (!dev || !dev->init_flag)
		return -1;

	CAN_Receive(dev->config.canx, fifo_number, msg);
	
	return 0;
}

/******************************************************************************
 * @brief	去初始化ADC
 * @param	dev	:  can_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __can_deinit(can_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
