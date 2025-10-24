#include "exti.h"

#ifdef USE_STDPERIPH_DRIVER

/**************************** GD32 系列 ****************************/
#if defined(GD32F10X_MD) || defined(GD32F10X_HD)

#define exti_get_irqn(pin) ((pin) == GPIO_PIN_0  ? EXTI0_IRQn :     \
						   ((pin) == GPIO_PIN_1  ? EXTI1_IRQn :     \
						   ((pin) == GPIO_PIN_2  ? EXTI2_IRQn :     \
						   ((pin) == GPIO_PIN_3  ? EXTI3_IRQn :     \
						   ((pin) == GPIO_PIN_4  ? EXTI4_IRQn :     \
						   ((pin) == GPIO_PIN_5  ? EXTI5_9_IRQn :   \
						   ((pin) == GPIO_PIN_6  ? EXTI5_9_IRQn :   \
						   ((pin) == GPIO_PIN_7  ? EXTI5_9_IRQn :   \
						   ((pin) == GPIO_PIN_8  ? EXTI5_9_IRQn :   \
						   ((pin) == GPIO_PIN_9  ? EXTI5_9_IRQn :   \
						   ((pin) == GPIO_PIN_10 ? EXTI10_15_IRQn :	\
						   ((pin) == GPIO_PIN_11 ? EXTI10_15_IRQn :	\
						   ((pin) == GPIO_PIN_12 ? EXTI10_15_IRQn :	\
						   ((pin) == GPIO_PIN_13 ? EXTI10_15_IRQn :	\
						   ((pin) == GPIO_PIN_14 ? EXTI10_15_IRQn :	\
						   ((pin) == GPIO_PIN_15 ? EXTI10_15_IRQn : (IRQn_Type)0))))))))))))))))

#define exti_get_line(pin) ((pin) == GPIO_PIN_0  ? EXTI_0 :  \
						   ((pin) == GPIO_PIN_1  ? EXTI_1 :  \
						   ((pin) == GPIO_PIN_2  ? EXTI_2 :  \
						   ((pin) == GPIO_PIN_3  ? EXTI_3 :  \
						   ((pin) == GPIO_PIN_4  ? EXTI_4 :  \
						   ((pin) == GPIO_PIN_5  ? EXTI_5 :  \
						   ((pin) == GPIO_PIN_6  ? EXTI_6 :  \
						   ((pin) == GPIO_PIN_7  ? EXTI_7 :  \
						   ((pin) == GPIO_PIN_8  ? EXTI_8 :  \
						   ((pin) == GPIO_PIN_9  ? EXTI_9 :  \
						   ((pin) == GPIO_PIN_10 ? EXTI_10 : \
						   ((pin) == GPIO_PIN_11 ? EXTI_11 : \
						   ((pin) == GPIO_PIN_12 ? EXTI_12 : \
						   ((pin) == GPIO_PIN_13 ? EXTI_13 : \
						   ((pin) == GPIO_PIN_14 ? EXTI_14 : \
						   ((pin) == GPIO_PIN_15 ? EXTI_15 : (exti_line_enum)0))))))))))))))))

#define exti_get_index(pin) ((pin) == GPIO_PIN_0  ? 0 :  \
						    ((pin) == GPIO_PIN_1  ? 1 :  \
						    ((pin) == GPIO_PIN_2  ? 2 :  \
						    ((pin) == GPIO_PIN_3  ? 3 :  \
						    ((pin) == GPIO_PIN_4  ? 4 :  \
						    ((pin) == GPIO_PIN_5  ? 5 :  \
						    ((pin) == GPIO_PIN_6  ? 6 :  \
						    ((pin) == GPIO_PIN_7  ? 7 :  \
						    ((pin) == GPIO_PIN_8  ? 8 :  \
						    ((pin) == GPIO_PIN_9  ? 9 :  \
						    ((pin) == GPIO_PIN_10 ? 10 : \
						    ((pin) == GPIO_PIN_11 ? 11 : \
						    ((pin) == GPIO_PIN_12 ? 12 : \
						    ((pin) == GPIO_PIN_13 ? 13 : \
						    ((pin) == GPIO_PIN_14 ? 14 : \
						    ((pin) == GPIO_PIN_15 ? 15 : (int)-1))))))))))))))))

	/**************************** GD32F1 系列 ****************************/
	#if defined(GD32F10X_MD) || defined(GD32F10X_HD)

	#define exti_io_clock_enable(port)									 \
		do {                                           				     \
			if (port == GPIOA)       rcu_periph_clock_enable(RCU_GPIOA); \
			else if (port == GPIOB)  rcu_periph_clock_enable(RCU_GPIOB); \
			else if (port == GPIOC)  rcu_periph_clock_enable(RCU_GPIOC); \
			else if (port == GPIOD)  rcu_periph_clock_enable(RCU_GPIOD); \
			else if (port == GPIOE)  rcu_periph_clock_enable(RCU_GPIOE); \
			else if (port == GPIOF)  rcu_periph_clock_enable(RCU_GPIOF); \
			else if (port == GPIOG)  rcu_periph_clock_enable(RCU_GPIOG); \
		} while (0)

	#define exti_config_io_in_pu(port, pin) \
			gpio_init(port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pin)

	#define exti_config_io_in_pd(port, pin) \
			gpio_init(port, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, pin)

	#define exti_config_io_in_pn(port, pin)	\
			gpio_init(port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, pin)

	#define exti_read_io_in_bit(port, pin)	gpio_input_bit_get(port, pin)

	#define	exti_get_gpio_port_source(port)	((port) == GPIOA ? GPIO_PORT_SOURCE_GPIOA : \
											((port) == GPIOB ? GPIO_PORT_SOURCE_GPIOB : \
											((port) == GPIOC ? GPIO_PORT_SOURCE_GPIOC : \
											((port) == GPIOD ? GPIO_PORT_SOURCE_GPIOD : \
											((port) == GPIOE ? GPIO_PORT_SOURCE_GPIOE : \
											((port) == GPIOF ? GPIO_PORT_SOURCE_GPIOF : \
											((port) == GPIOG ? GPIO_PORT_SOURCE_GPIOG : (int)-1)))))))

	#define	exti_get_gpio_pin_source(pin) ((pin) == GPIO_PIN_0  ? GPIO_PIN_SOURCE_0 :  \
										  ((pin) == GPIO_PIN_1  ? GPIO_PIN_SOURCE_1 :  \
										  ((pin) == GPIO_PIN_2  ? GPIO_PIN_SOURCE_2 :  \
										  ((pin) == GPIO_PIN_3  ? GPIO_PIN_SOURCE_3 :  \
										  ((pin) == GPIO_PIN_4  ? GPIO_PIN_SOURCE_4 :  \
										  ((pin) == GPIO_PIN_5  ? GPIO_PIN_SOURCE_5 :  \
										  ((pin) == GPIO_PIN_6  ? GPIO_PIN_SOURCE_6 :  \
										  ((pin) == GPIO_PIN_7  ? GPIO_PIN_SOURCE_7 :  \
										  ((pin) == GPIO_PIN_8  ? GPIO_PIN_SOURCE_8 :  \
										  ((pin) == GPIO_PIN_9  ? GPIO_PIN_SOURCE_9 :  \
										  ((pin) == GPIO_PIN_10 ? GPIO_PIN_SOURCE_10 : \
										  ((pin) == GPIO_PIN_11 ? GPIO_PIN_SOURCE_11 : \
										  ((pin) == GPIO_PIN_12 ? GPIO_PIN_SOURCE_12 : \
										  ((pin) == GPIO_PIN_13 ? GPIO_PIN_SOURCE_13 : \
										  ((pin) == GPIO_PIN_14 ? GPIO_PIN_SOURCE_14 : \
										  ((pin) == GPIO_PIN_15 ? GPIO_PIN_SOURCE_15 : (int)-1))))))))))))))))

	#endif	/* GD32F1 系列 */

#endif	/* GD32 系列 */

#endif	/* USE_STDPERIPH_DRIVER */

/* EXTI设备指针数组 */
exti_dev_t *g_exti_dev[16] = {NULL};

/* 函数声明 */
static int exti_drv_deinit(exti_dev_t *dev);

/**
 * @brief   初始化 EXTI
 * @param[in,out] dev exti_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int exti_drv_init(exti_dev_t *dev)
{
	if (!dev)
		return -1;

	int8_t index = exti_get_index(dev->config.pin);

	if (index < 0 || index >= 16)
		return -2;

	if (g_exti_dev[index] != NULL)
    	return -3;  // 已注册

	/* 开启时钟与配置 GPIO */
	rcu_periph_clock_enable(RCU_AF);	// 开启 AFIO 时钟
	exti_io_clock_enable(dev->config.port);

	if (dev->config.trigger == EXTI_TRIG_FALLING)
		exti_config_io_in_pu(dev->config.port, dev->config.pin);
	else if (dev->config.trigger == EXTI_TRIG_RISING)
		exti_config_io_in_pd(dev->config.port, dev->config.pin);
	else if (dev->config.trigger == EXTI_TRIG_BOTH)
		exti_config_io_in_pn(dev->config.port, dev->config.pin);
	else
		return -4;

	/* 配置 AFIO */
	gpio_exti_source_select(exti_get_gpio_port_source(dev->config.port), 
							exti_get_gpio_pin_source(dev->config.pin));

	/* 配置 EXTI */
	exti_init(exti_get_line(dev->config.pin), EXTI_INTERRUPT, dev->config.trigger);
	exti_interrupt_flag_clear(exti_get_line(dev->config.pin));
	
	/* 配置中断 */
	nvic_irq_enable(exti_get_irqn(dev->config.pin), 
					dev->config.pre_priority, 
					dev->config.sub_priority);
	exti_interrupt_enable(exti_get_line(dev->config.pin));	// exti_init 中已设置，可省略

	
	g_exti_dev[index] = dev;	// 记录设备指针用于中断处理

	dev->deinit = exti_drv_deinit;
	dev->init_flag = true;
	return 0;
}

/**
 * @brief   去初始化 EXTI
 * @param[in,out] dev exti_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
static int exti_drv_deinit(exti_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	int8_t index = exti_get_index(dev->config.pin);

	if (index < 0 || index >= 16)
		return -2;
	
	g_exti_dev[index] = NULL;
	dev->init_flag = false;
	
	return 0;
}

/**
 * @brief   通用中断处理函数，内部使用
 * @param[in] dev exti_dev_t 结构体指针
 */	
static void exti_irq_handler(exti_dev_t *dev)
{
	if (exti_interrupt_flag_get(exti_get_line(dev->config.pin)) == SET) {
		if (dev->config.trigger == EXTI_TRIG_FALLING && 
			exti_read_io_in_bit(dev->config.port, dev->config.pin) == GPIO_LEVEL_LOW) {
            /* 下降沿触发且当前为低电平 */
			exti_interrupt_flag_clear(exti_get_line(dev->config.pin));	// 清除中断标志位
			if (dev->config.falling_irq_handler != NULL)
				dev->config.falling_irq_handler();
		
		} else if (dev->config.trigger == EXTI_TRIG_RISING && 
				   exti_read_io_in_bit(dev->config.port, dev->config.pin) == GPIO_LEVEL_HIGH) {
			/* 上升沿触发且当前为高电平 */
			exti_interrupt_flag_clear(exti_get_line(dev->config.pin));	// 清除中断标志位
			if (dev->config.rising_irq_handler != NULL)
				dev->config.rising_irq_handler();
		
		} else if (dev->config.trigger == EXTI_TRIG_BOTH) {
            if (exti_read_io_in_bit(dev->config.port, dev->config.pin) == GPIO_LEVEL_LOW) {
                /* 双边沿触发且当前为低电平 */
                exti_interrupt_flag_clear(exti_get_line(dev->config.pin));	// 清除中断标志位
                if (dev->config.falling_irq_handler != NULL)
                    dev->config.falling_irq_handler();
            } else {
                /* 双边沿触发且当前为高电平 */
                exti_interrupt_flag_clear(exti_get_line(dev->config.pin));	// 清除中断标志位
                if (dev->config.rising_irq_handler != NULL)
                    dev->config.rising_irq_handler();
            }
		
		} else {
			return;
		}
	}
}

/**
 * @brief   EXTI0 中断函数
 */
void EXTI0_IRQHandler(void)
{
	if (g_exti_dev[0] != NULL)
		exti_irq_handler(g_exti_dev[0]);
}

/**
 * @brief   EXTI1 中断函数
 */
void EXTI1_IRQHandler(void)
{
	if (g_exti_dev[1] != NULL)
		exti_irq_handler(g_exti_dev[1]);
}

/**
 * @brief   EXTI2 中断函数
 */
void EXTI2_IRQHandler(void)
{
	if (g_exti_dev[2] != NULL)
		exti_irq_handler(g_exti_dev[2]);
}

/**
 * @brief   EXTI3 中断函数
 */
void EXTI3_IRQHandler(void)
{
	if (g_exti_dev[3] != NULL)
		exti_irq_handler(g_exti_dev[3]);
}

/**
 * @brief   EXTI4 中断函数
 */
void EXTI4_IRQHandler(void)
{
	if (g_exti_dev[4] != NULL)
		exti_irq_handler(g_exti_dev[4]);
}

/**
 * @brief   EXTI5_9 中断函数
 */
void EXTI5_9_IRQHandler(void)
{
	for (int i = 5; i <= 9; i++)
		if (g_exti_dev[i] != NULL)
			exti_irq_handler(g_exti_dev[i]);
}

/**
 * @brief   EXTI10_15 中断函数
 */
void EXTI10_15_IRQHandler(void)
{
	for (int i = 10; i <= 15; i++)
		if (g_exti_dev[i] != NULL)
			exti_irq_handler(g_exti_dev[i]);
}
