#include "exti.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_MD) || defined(STM32F10X_HD) || defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__exti_get_gpio_port_source(port)	(	port == GPIOA ? GPIO_PortSourceGPIOA : \
												port == GPIOB ? GPIO_PortSourceGPIOB : \
												port == GPIOC ? GPIO_PortSourceGPIOC : \
												port == GPIOD ? GPIO_PortSourceGPIOD : \
												port == GPIOE ? GPIO_PortSourceGPIOE : \
												port == GPIOF ? GPIO_PortSourceGPIOF : \
												port == GPIOG ? GPIO_PortSourceGPIOG : \
												(int)-1	)
															
#define	__exti_get_gpio_pin_source(pin)	(	pin == GPIO_Pin_0 ? GPIO_PinSource0 : \
											pin == GPIO_Pin_1 ? GPIO_PinSource1 : \
											pin == GPIO_Pin_2 ? GPIO_PinSource2 : \
											pin == GPIO_Pin_3 ? GPIO_PinSource3 : \
											pin == GPIO_Pin_4 ? GPIO_PinSource4 : \
											pin == GPIO_Pin_5 ? GPIO_PinSource5 : \
											pin == GPIO_Pin_6 ? GPIO_PinSource6 : \
											pin == GPIO_Pin_7 ? GPIO_PinSource7 : \
											pin == GPIO_Pin_8 ? GPIO_PinSource8 : \
											pin == GPIO_Pin_9 ? GPIO_PinSource9 : \
											pin == GPIO_Pin_10 ? GPIO_PinSource10 : \
											pin == GPIO_Pin_11 ? GPIO_PinSource11 : \
											pin == GPIO_Pin_12 ? GPIO_PinSource12 : \
											pin == GPIO_Pin_13 ? GPIO_PinSource13 : \
											pin == GPIO_Pin_14 ? GPIO_PinSource14 : \
											pin == GPIO_Pin_15 ? GPIO_PinSource15 : \
											(int)-1	)
												
#define	__exti_nvic_get_irq_channel(pin)	(	pin == GPIO_Pin_0 ? EXTI0_IRQn : \
												pin == GPIO_Pin_1 ? EXTI1_IRQn : \
												pin == GPIO_Pin_2 ? EXTI2_IRQn : \
												pin == GPIO_Pin_3 ? EXTI3_IRQn : \
												pin == GPIO_Pin_4 ? EXTI4_IRQn : \
												pin == GPIO_Pin_5 ? EXTI9_5_IRQn : \
												pin == GPIO_Pin_6 ? EXTI9_5_IRQn : \
												pin == GPIO_Pin_7 ? EXTI9_5_IRQn : \
												pin == GPIO_Pin_8 ? EXTI9_5_IRQn : \
												pin == GPIO_Pin_9 ? EXTI9_5_IRQn : \
												pin == GPIO_Pin_10 ? EXTI15_10_IRQn : \
												pin == GPIO_Pin_11 ? EXTI15_10_IRQn : \
												pin == GPIO_Pin_12 ? EXTI15_10_IRQn : \
												pin == GPIO_Pin_13 ? EXTI15_10_IRQn : \
												pin == GPIO_Pin_14 ? EXTI15_10_IRQn : \
												pin == GPIO_Pin_15 ? EXTI15_10_IRQn : \
												(int)-1	)

#define	__exti_get_line(pin)	(	pin == GPIO_Pin_0 ? EXTI_Line0 : \
									pin == GPIO_Pin_1 ? EXTI_Line1 : \
									pin == GPIO_Pin_2 ? EXTI_Line2 : \
									pin == GPIO_Pin_3 ? EXTI_Line3 : \
									pin == GPIO_Pin_4 ? EXTI_Line4 : \
									pin == GPIO_Pin_5 ? EXTI_Line5 : \
									pin == GPIO_Pin_6 ? EXTI_Line6 : \
									pin == GPIO_Pin_7 ? EXTI_Line7 : \
									pin == GPIO_Pin_8 ? EXTI_Line8 : \
									pin == GPIO_Pin_9 ? EXTI_Line9 : \
									pin == GPIO_Pin_10 ? EXTI_Line10 : \
									pin == GPIO_Pin_11 ? EXTI_Line11 : \
									pin == GPIO_Pin_12 ? EXTI_Line12 : \
									pin == GPIO_Pin_13 ? EXTI_Line13 : \
									pin == GPIO_Pin_14 ? EXTI_Line14 : \
									pin == GPIO_Pin_15 ? EXTI_Line15 : \
									(int)0x10000)									

#define	__exti_get_index(pin)	(	pin == GPIO_Pin_0 ? 0 : \
									pin == GPIO_Pin_1 ? 1 : \
									pin == GPIO_Pin_2 ? 2 : \
									pin == GPIO_Pin_3 ? 3 : \
									pin == GPIO_Pin_4 ? 4 : \
									pin == GPIO_Pin_5 ? 5 : \
									pin == GPIO_Pin_6 ? 6 : \
									pin == GPIO_Pin_7 ? 7 : \
									pin == GPIO_Pin_8 ? 8 : \
									pin == GPIO_Pin_9 ? 9 : \
									pin == GPIO_Pin_10 ? 10 : \
									pin == GPIO_Pin_11 ? 11 : \
									pin == GPIO_Pin_12 ? 12 : \
									pin == GPIO_Pin_13 ? 13 : \
									pin == GPIO_Pin_14 ? 14 : \
									pin == GPIO_Pin_15 ? 15 : \
									(int)-1)

	#if defined(STM32F10X_MD) || defined(STM32F10X_HD)

	#define	__exti_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
												else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
												else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
												else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
												else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
												else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
												else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
											}

	#define	__exti_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

	#define	__exti_config_io_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

	#define	__exti_config_io_in_pn(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

	#define __exti_read_io_in_bit(port, pin)	GPIO_ReadInputDataBit(port, pin)

	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

	#define	__exti_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
												else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
												else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
												else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
												else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
												else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
												else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
											}

	#define	__exti_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

	#define	__exti_config_io_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

	#define	__exti_config_io_in_pn(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

	#define __exti_read_io_in_bit(port, pin)	GPIO_ReadInputDataBit(port, pin)

	#endif

#endif

#endif

/* EXTI设备指针数组 */
exti_dev_t *g_exti_dev[16] = {NULL};

/* 函数声明 */
static int8_t __exti_deinit(exti_dev_t *dev);

/******************************************************************************
 * @brief	EXTI初始化
 * @param	dev	:	exti_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t exti_init(exti_dev_t *dev)
{
	if (!dev)
		return -1;

	int8_t index = __exti_get_index(dev->config.pin);

	if (index < 0 || index >= 16)
		return -2;

	if (g_exti_dev[index] != NULL)
    	return -3;  // 已经被注册
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	/* 开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	// 开启AFIO时钟
	__exti_io_clock_enable(dev->config.port);

	/* 配置AFIO */
	GPIO_EXTILineConfig(__exti_get_gpio_port_source(dev->config.port), __exti_get_gpio_pin_source(dev->config.pin));
	
	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

	/* 开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// 开启SYSCFG时钟
	__exti_io_clock_enable(dev->config.port);

	/* 配置SYSCFG */
	SYSCFG_EXTILineConfig(__exti_get_gpio_port_source(dev->config.port), __exti_get_gpio_pin_source(dev->config.pin));
	
	#endif

	/* 配置GPIO */
	if (dev->config.trigger == EXTI_Trigger_Falling)
	{
		__exti_config_io_in_pu(dev->config.port, dev->config.pin);
	}
	else if (dev->config.trigger == EXTI_Trigger_Rising)
	{
		__exti_config_io_in_pd(dev->config.port, dev->config.pin);
	}
	else if (dev->config.trigger == EXTI_Trigger_Rising_Falling)
	{
		__exti_config_io_in_pn(dev->config.port, dev->config.pin);
	}
	else
	{
		return -4;
	}

	/* 配置EXTI */
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = __exti_get_line(dev->config.pin);
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = dev->config.trigger;
	EXTI_Init(&EXTI_InitStructure);
	
	/* 配置NVIC */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = __exti_nvic_get_irq_channel(dev->config.pin);
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = dev->config.preemption_priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = dev->config.sub_priority;
	NVIC_Init(&NVIC_InitStructure);

	/* 记录设备指针用于中断处理 */
	g_exti_dev[index] = dev;

	/* 函数指针赋值 */
	dev->deinit = __exti_deinit;
	
	dev->init_flag = true;

	return 0;
}

/******************************************************************************
 * @brief	去初始化EXTI
 * @param	dev   :  exti_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __exti_deinit(exti_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	int8_t index = __exti_get_index(dev->config.pin);

	if (index < 0 || index >= 16)
		return -2;
	
	g_exti_dev[index] = NULL;
	
	dev->init_flag = false;
	
	return 0;
}

/******************************************************************************
 * @brief	通用中断处理函数，内部使用
 * @param	dev	:  exti_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __exti_irq_handler(exti_dev_t *dev)
{
	if (EXTI_GetITStatus(__exti_get_line(dev->config.pin)) == SET)
	{
		if (dev->config.trigger == EXTI_Trigger_Falling && 
			__exti_read_io_in_bit(dev->config.port, dev->config.pin) == GPIO_LEVEL_LOW)
		{
            /* 下降沿触发且当前为低电平 */
			EXTI_ClearITPendingBit(__exti_get_line(dev->config.pin));	// 清除中断标志位
			if (dev->config.falling_irq_handler != NULL)
			{
				dev->config.falling_irq_handler();
			}
		}
		else if (dev->config.trigger == EXTI_Trigger_Rising && 
				__exti_read_io_in_bit(dev->config.port, dev->config.pin) == GPIO_LEVEL_HIGH)
		{
			/* 上升沿触发且当前为高电平 */
			EXTI_ClearITPendingBit(__exti_get_line(dev->config.pin));	// 清除中断标志位
			if (dev->config.rising_irq_handler != NULL)
			{
				dev->config.rising_irq_handler();
			}
		}
		else if (dev->config.trigger == EXTI_Trigger_Rising_Falling)
		{
            if (__exti_read_io_in_bit(dev->config.port, dev->config.pin) == GPIO_LEVEL_LOW)
            {
                /* 双边沿触发且当前为低电平 */
                EXTI_ClearITPendingBit(__exti_get_line(dev->config.pin));	// 清除中断标志位
                if (dev->config.falling_irq_handler != NULL)
                {
                    dev->config.falling_irq_handler();
                }
            }
            else
            {
                /* 双边沿触发且当前为低电平 */
                EXTI_ClearITPendingBit(__exti_get_line(dev->config.pin));	// 清除中断标志位
                if (dev->config.rising_irq_handler != NULL)
                {
                    dev->config.rising_irq_handler();
                }
            }
		}
		else
		{
			return;
		}
	}
}

/******************************************************************************
 * @brief	EXTI0中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void EXTI0_IRQHandler(void)
{
	if (g_exti_dev[0] != NULL)
	{
		__exti_irq_handler(g_exti_dev[0]);
	}
}

/******************************************************************************
 * @brief	EXTI1中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void EXTI1_IRQHandler(void)
{
	if (g_exti_dev[1] != NULL)
	{
		__exti_irq_handler(g_exti_dev[1]);
	}
}

/******************************************************************************
 * @brief	EXTI2中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void EXTI2_IRQHandler(void)
{
	if (g_exti_dev[2] != NULL)
	{
		__exti_irq_handler(g_exti_dev[2]);
	}
}

/******************************************************************************
 * @brief	EXTI3中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void EXTI3_IRQHandler(void)
{
	if (g_exti_dev[3] != NULL)
	{
		__exti_irq_handler(g_exti_dev[3]);
	}
}

/******************************************************************************
 * @brief	EXTI4中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void EXTI4_IRQHandler(void)
{
	if (g_exti_dev[4] != NULL)
	{
		__exti_irq_handler(g_exti_dev[4]);
	}
}

/******************************************************************************
 * @brief	EXTI9_5中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	if (g_exti_dev[5] != NULL)
	{
		__exti_irq_handler(g_exti_dev[5]);
	}
	if (g_exti_dev[6] != NULL)
	{
		__exti_irq_handler(g_exti_dev[6]);
	}
	if (g_exti_dev[7] != NULL)
	{
		__exti_irq_handler(g_exti_dev[7]);
	}
	if (g_exti_dev[8] != NULL)
	{
		__exti_irq_handler(g_exti_dev[8]);
	}
	if (g_exti_dev[9] != NULL)
	{
    	__exti_irq_handler(g_exti_dev[9]);
	}
}

/******************************************************************************
 * @brief	EXTI15_10中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
	if (g_exti_dev[10] != NULL)
	{
		__exti_irq_handler(g_exti_dev[10]);
	}
	if (g_exti_dev[11] != NULL)
	{
		__exti_irq_handler(g_exti_dev[11]);
	}
	if (g_exti_dev[12] != NULL)
	{
		__exti_irq_handler(g_exti_dev[12]);
	}
	if (g_exti_dev[13] != NULL)
	{
		__exti_irq_handler(g_exti_dev[13]);
	}
	if (g_exti_dev[14] != NULL)
	{
		__exti_irq_handler(g_exti_dev[14]);
	}
	if (g_exti_dev[15] != NULL)
	{
		__exti_irq_handler(g_exti_dev[15]);
	}
}
