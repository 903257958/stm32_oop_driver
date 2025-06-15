#include "encoder.h"

#if defined(STM32F10X_MD) || defined(STM32F10X_HD) || \
	defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define __exti_read_io_in_bit(port, pin)	GPIO_ReadInputDataBit(port, pin)

#endif

/* 旋转编码器设备指针数组 */
encoder_dev_t *g_encoder_dev[MAX_ENCODER_NUM];
uint8_t g_encoder_num;

/* 函数指针数组，保存用户注册的回调函数 */
static encoder_callback_t g_encoder_forward_callback[MAX_ENCODER_NUM] = {0};
static encoder_callback_t g_encoder_reverse_callback[MAX_ENCODER_NUM] = {0};

/* 空指针数组，保存用户注册的回调函数参数 */
static void *g_encoder_forward_callback_param[MAX_ENCODER_NUM] = {NULL};
static void *g_encoder_reverse_callback_param[MAX_ENCODER_NUM] = {NULL};

/* 旋转编码器私有数据结构体 */
typedef struct {
	exti_dev_t s1;
	exti_dev_t s2;
} encoder_priv_data_t;

/* 函数声明 */
static void __encoder_s1_irq_handler(void);	// 旋转编码器S1中断处理函数，内部调用用户注册的正转回调函数
static void __encoder_s2_irq_handler(void);	// 旋转编码器S2中断处理函数，内部调用用户注册的反转回调函数
static int8_t __encoder_deinit(encoder_dev_t *dev);

/******************************************************************************
 * @brief	旋转编码器初始化
 * @param	dev	:	encoder_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t encoder_init(encoder_dev_t *dev)
{
	if (!dev)
		return -1;

	if (g_encoder_num >= MAX_ENCODER_NUM)
		return -2;	// 初始化前发现已注册的设备已经等于最大设备数，初始化失败

	uint8_t index = g_encoder_num;

	/* 初始化私有数据 */
	dev->priv_data = (encoder_priv_data_t *)malloc(sizeof(encoder_priv_data_t));
	if (!dev->priv_data)
		return -3;
	
	encoder_priv_data_t *priv_data = (encoder_priv_data_t *)dev->priv_data;

	/* 保存用户注册的回调函数指针与参数 */
	g_encoder_forward_callback[index] = dev->config.forward_callback;
	g_encoder_reverse_callback[index] = dev->config.reverse_callback;
	g_encoder_forward_callback_param[index] = dev->config.forward_callback_param;
	g_encoder_reverse_callback_param[index] = dev->config.reverse_callback_param;

	/* 记录设备指针用于中断处理 */
	g_encoder_dev[index] = dev;

	/* 外部中断初始化 */
	priv_data->s1.config.port = dev->config.s1_port;
	priv_data->s1.config.pin = dev->config.s1_pin;
	priv_data->s1.config.trigger = EXTI_Trigger_Falling;
	priv_data->s1.config.preemption_priority = 1;
	priv_data->s1.config.sub_priority = 0;
	priv_data->s1.config.falling_irq_handler = __encoder_s1_irq_handler;
	priv_data->s1.config.rising_irq_handler = NULL;

	priv_data->s2.config.port = dev->config.s2_port;
	priv_data->s2.config.pin = dev->config.s2_pin;
	priv_data->s2.config.trigger = EXTI_Trigger_Falling;
	priv_data->s2.config.preemption_priority = 1;
	priv_data->s2.config.sub_priority = 0;
	priv_data->s2.config.falling_irq_handler = __encoder_s2_irq_handler;
	priv_data->s2.config.rising_irq_handler = NULL;

	exti_init(&priv_data->s1);
	exti_init(&priv_data->s2);

	/* 旋转编码器设备数加1 */
	g_encoder_num++;
	
	/* 函数指针赋值 */
	dev->deinit = __encoder_deinit;

	dev->init_flag = true;

	return 0;
}

/******************************************************************************
 * @brief	旋转编码器S1中断处理函数，内部使用
 * @param	无
 * @return	无
 ******************************************************************************/
static void __encoder_s1_irq_handler(void)
{
	uint8_t i;

	/* 遍历所有旋转编码器设备 */
	for (i = 0; i < g_encoder_num; i++)
	{
		/* 已进入S1下降沿触发中断，此时检测S2的电平，判断旋转方向 */
		if (__exti_read_io_in_bit(g_encoder_dev[i]->config.s2_port, g_encoder_dev[i]->config.s2_pin) == GPIO_LEVEL_LOW)
		{
			/* 调用用户注册的正转回调函数 */
			if (g_encoder_forward_callback[i])
			{
				g_encoder_forward_callback[i](g_encoder_forward_callback_param[i]);
			}
		}
	}
}

/******************************************************************************
 * @brief	旋转编码器S2中断处理函数，内部使用
 * @param	无
 * @return	无
 ******************************************************************************/
static void __encoder_s2_irq_handler(void)
{
	uint8_t i;

	/* 遍历所有旋转编码器设备 */
	for (i = 0; i < g_encoder_num; i++)
	{
		/* 已进入S2下降沿触发中断，此时检测S1的电平，判断旋转方向 */
		if (__exti_read_io_in_bit(g_encoder_dev[i]->config.s1_port, g_encoder_dev[i]->config.s1_pin) == GPIO_LEVEL_LOW)
		{
			/* 调用用户注册的反转回调函数 */
			if (g_encoder_reverse_callback[i])
			{
				g_encoder_reverse_callback[i](g_encoder_reverse_callback_param[i]);
			}
		}
	}
}

/******************************************************************************
 * @brief	去初始化旋转编码器
 * @param	dev   :  encoder_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __encoder_deinit(encoder_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	encoder_priv_data_t *priv_data = (encoder_priv_data_t *)dev->priv_data;

	/* 去初始化外部中断 */
	priv_data->s1.deinit(&priv_data->s1);
	priv_data->s2.deinit(&priv_data->s2);

	free(priv_data);
	dev->priv_data = NULL;
	dev->init_flag = false;
	
	return 0;
}
