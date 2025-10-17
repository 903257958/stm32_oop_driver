#include "sr04.h"
#include <stdlib.h>

#ifdef USE_STDPERIPH_DRIVER

/**************************** GD32F1 系列 ****************************/
#if defined(GD32F10X_MD)

#define TIMER_FREQ	108000000

#define sr04_timer_clock_enable(timerx)									\
    do {                                           				        \
        if (timerx == TIMER1)      rcu_periph_clock_enable(RCU_TIMER1); \
        else if (timerx == TIMER2) rcu_periph_clock_enable(RCU_TIMER2); \
        else if (timerx == TIMER3) rcu_periph_clock_enable(RCU_TIMER3); \
    } while (0)

#define sr04_io_clock_enable(port)									 \
    do {                                           				     \
        if (port == GPIOA)       rcu_periph_clock_enable(RCU_GPIOA); \
        else if (port == GPIOB)  rcu_periph_clock_enable(RCU_GPIOB); \
        else if (port == GPIOC)  rcu_periph_clock_enable(RCU_GPIOC); \
        else if (port == GPIOD)  rcu_periph_clock_enable(RCU_GPIOD); \
        else if (port == GPIOE)  rcu_periph_clock_enable(RCU_GPIOE); \
        else if (port == GPIOF)  rcu_periph_clock_enable(RCU_GPIOF); \
        else if (port == GPIOG)  rcu_periph_clock_enable(RCU_GPIOG); \
    } while (0)

#define sr04_config_io_out_pp(port, pin) \
        gpio_init(port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin)

#define sr04_config_io_in_pu(port, pin) \
    	gpio_init(port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pin)

#define	sr04_gpio_write_bit(port, pin, value) gpio_bit_write(port, pin, (bit_status)value)

#define sr04_timer_get_irqn(timerx) ((timerx) == TIMER1 ? TIMER1_IRQn : \
							        ((timerx) == TIMER2 ? TIMER2_IRQn : \
							        ((timerx) == TIMER3 ? TIMER3_IRQn : (IRQn_Type)0)))

#define timer_get_index(timerx)	((timerx) == TIMER1 ? 0 : \
								((timerx) == TIMER2 ? 1 : \
								((timerx) == TIMER3 ? 2 : (int)-1)))                
#endif	/* GD32F1 系列 */

#endif	/* USE_STDPERIPH_DRIVER */

/* SR04设备指针数组 */
sr04_dev_t *g_sr04_dev[MAX_SR04_NUM] = {NULL};
uint8_t g_sr04_num;

/* SR04私有数据结构体 */
typedef struct {
	volatile uint16_t t1;	// 捕获的上升沿时间（Echo开始变高）
	volatile uint16_t t2;	// 捕获的下降沿时间（Echo结束变低）
	volatile bool is_first;	// 标记当前是第一次捕获（上升沿）还是第二次捕获（下降沿）
	volatile bool done;		// 捕获过程是否完成
} sr04_priv_data_t;

/* 函数声明 */
static void sr04_trigger(sr04_dev_t *dev);
static int sr04_get_distance(sr04_dev_t *dev);
static int sr04_deinit(sr04_dev_t *dev);
static void sr04_timx_irq_callback(timer_periph_t timx);

/**
 * @brief   初始化 SR04 设备
 * @param[in,out] dev sr04_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
int sr04_init(sr04_dev_t *dev)
{
	if (!dev)
		return -1;

	if (g_sr04_num >= MAX_SR04_NUM)
		return -2;

	uint8_t index = g_sr04_num;
	g_sr04_dev[index] = dev;	// 记录设备指针用于中断处理

	dev->priv_data = (sr04_priv_data_t *)malloc(sizeof(sr04_priv_data_t));
	if (!dev->priv_data)
		return -3;
	
	sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;
	priv_data->t1 = 0;
	priv_data->t2 = 0;
	priv_data->is_first = true;
	priv_data->done = false;

	sr04_timer_clock_enable(dev->config.timx);
	sr04_io_clock_enable(dev->config.trig_port);
	sr04_io_clock_enable(dev->config.echo_port);
	sr04_config_io_out_pp(dev->config.trig_port, dev->config.trig_pin);	// Trig 推挽输出
	sr04_config_io_in_pu(dev->config.echo_port, dev->config.echo_pin);	// Echo 上拉输入

	/* 配置时钟源为内部时钟 */
	timer_internal_clock_config(dev->config.timx);

	/* 时基单元初始化 */
	timer_parameter_struct timer_initpara;
	timer_initpara.clockdivision = TIMER_CKDIV_DIV1;		// 时钟分频参数，不分频
	timer_initpara.alignedmode = TIMER_COUNTER_EDGE;		// 对齐模式
	timer_initpara.counterdirection = TIMER_COUNTER_UP;		// 计数器模式：向上计数
	timer_initpara.prescaler = TIMER_FREQ / 1000000 - 1;	// PSC预分频器的值，计数频率1MHz，计数周期1us
	timer_initpara.period = 0xFFFF;							// ARR自动重装器的值，范围0~65535
	timer_initpara.repetitioncounter = 0;					// 重复计数器的值（高级定时器用）
	timer_init(dev->config.timx, &timer_initpara);

    /* 初始化输入捕获 */
	timer_ic_parameter_struct timer_ic_parameter_icpara;
	timer_ic_parameter_icpara.icfilter = 0x0;								// 输入滤波器参数
	timer_ic_parameter_icpara.icpolarity = TIMER_IC_POLARITY_RISING;		// 极性：上升沿触发捕获
	timer_ic_parameter_icpara.icprescaler = TIMER_IC_PSC_DIV1;				// 捕获预分频：不分频，每次信号都触发捕获
	timer_ic_parameter_icpara.icselection = TIMER_IC_SELECTION_DIRECTTI;	// 输入信号交叉：直通，不交叉
	timer_input_capture_config(dev->config.timx, (uint16_t)dev->config.ic_channel, &timer_ic_parameter_icpara);

	/* 配置中断 */
	timer_interrupt_flag_clear(dev->config.timx,    // 启用前清除捕获标志，防止假触发
							   TIMER_INT_FLAG_UP  | 
							   TIMER_INT_FLAG_CH0 | 
							   TIMER_INT_FLAG_CH1 | 
							   TIMER_INT_FLAG_CH2 | 
							   TIMER_INT_FLAG_CH3);
	timer_interrupt_enable(dev->config.timx, TIMER_INT_CH0 << dev->config.ic_channel);
	nvic_irq_enable(sr04_timer_get_irqn(dev->config.timx), 0, 0);
    
    /* 启用定时器 */
	timer_enable(dev->config.timx);

	g_sr04_num++;

	dev->get_distance = sr04_get_distance;
	dev->deinit = sr04_deinit;
	
	dev->init_flag = true;
	return 0;
}

/**
 * @brief   SR04 触发测量，内部使用
 * @param[in] dev sr04_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */	
static void sr04_trigger(sr04_dev_t *dev)
{
	sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;

	sr04_gpio_write_bit(dev->config.trig_port, dev->config.trig_pin, GPIO_LEVEL_LOW);
    sr04_delay_us(2);
    sr04_gpio_write_bit(dev->config.trig_port, dev->config.trig_pin, GPIO_LEVEL_HIGH);
    sr04_delay_us(12);
    sr04_gpio_write_bit(dev->config.trig_port, dev->config.trig_pin, GPIO_LEVEL_LOW);

	priv_data->is_first = true;
	priv_data->done = false;
}

/**
 * @brief   SR04 获取距离值
 * @param[in] dev sr04_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int sr04_get_distance(sr04_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;
	uint8_t timeout_ms;
	uint16_t duration;

	/* 触发测量 */
	sr04_trigger(dev);

    /* 等待捕获完成，最多延迟30ms */
    timeout_ms = 30;
    while (!priv_data->done && timeout_ms--)
		sr04_delay_ms(1);

	/* 捕获完成，计算距离 */
    if (priv_data->done) {
        duration = (priv_data->t2 >= priv_data->t1) ? \
				   (priv_data->t2 - priv_data->t1) : \
				   (0xFFFF - priv_data->t1 + priv_data->t2);	// 处理定时器溢出

		dev->distance_cm = duration * 0.017f;  // 声速340m/s = 0.034cm/us，除以2表示往返距离
		return 0;
    }

	/* 超时或失败 */
	dev->distance_cm = -1.0f;
    return -2;
}

/**
 * @brief   去初始化 SR04 设备
 * @param[in,out] dev sr04_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int sr04_deinit(sr04_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;
	free(priv_data);
	dev->priv_data = NULL;
	dev->init_flag = false;
	
	return 0;
}

/**
 * @brief   SR04 定时器中断回调函数，内部使用
 * @param[in] timx 定时器外设
 */
static void sr04_timx_irq_callback(timer_periph_t timx)
{
	uint8_t i, ic_channel;
	uint16_t capture_val;

	/* 遍历找到与此定时器对应的SR04设备 */
	for (i = 0; i < g_sr04_num; i++) {
        sr04_dev_t *dev = g_sr04_dev[i];
        if (dev->config.timx != timx)
			continue;

        sr04_priv_data_t *priv_data = (sr04_priv_data_t *)dev->priv_data;
        ic_channel = dev->config.ic_channel;

        if (timer_interrupt_flag_get(timx, TIMER_INT_FLAG_CH0 << ic_channel) == SET) {
			timer_interrupt_flag_clear(timx, TIMER_INT_FLAG_CH0 << ic_channel);	// 清中断标志
            capture_val = timer_channel_capture_value_register_read(timx, (uint16_t)ic_channel);

            if (priv_data->is_first) {
				/* 当前是第一次捕获（上升沿） */
                priv_data->t1 = capture_val;	// 保存上升沿时间
				priv_data->is_first = false;	// 下次再次捕获时不是第一次捕获

                /* 切换极性：下次捕获下降沿 */
                timer_channel_output_polarity_config(timx, (uint16_t)ic_channel, TIMER_OC_POLARITY_LOW);

            } else {
				/* 当前是第二次捕获（下降沿） */
                priv_data->t2 = capture_val;	// 保存下降沿时间
                priv_data->done = true;			// 捕获完成
				priv_data->is_first = true;		// 下次再次捕获时是第一次捕获

                /* 重新切换为上升沿 */
                timer_channel_output_polarity_config(timx, (uint16_t)ic_channel, TIMER_OC_POLARITY_HIGH);
            }
        }
    }
}

#if SR04_TIM1_IRQ_HANDLER_ENABLE
/**
 * @brief   TIMER1 中断函数
 */
void TIMER1_IRQHandler(void)
{
	sr04_timx_irq_callback(TIMER1);
}
#endif

#if SR04_TIM2_IRQ_HANDLER_ENABLE
/**
 * @brief   TIMER2 中断函数
 */
void TIMER2_IRQHandler(void)
{
	sr04_timx_irq_callback(TIMER2);
}
#endif

#if SR04_TIM3_IRQ_HANDLER_ENABLE
/**
 * @brief   TIMER3 中断函数
 */
void TIMER3_IRQHandler(void)
{
    sr04_timx_irq_callback(TIMER3);
}
#endif
