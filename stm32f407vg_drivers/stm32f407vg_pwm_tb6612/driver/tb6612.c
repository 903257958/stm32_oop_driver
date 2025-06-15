#include "tb6612.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)

#define TIMER_FREQ	72000000

#define	__tb6612_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
											else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
											else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
											else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
											else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
											else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
											else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
										}

#define	__tb6612_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

#define	__gpio_write_bit(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#elif defined(STM32F40_41xxx) || defined(STM32F40_41xxx) || defined(STM32F429_439xx)

	#if defined(STM32F40_41xxx) || defined(STM32F40_41xxx)

	#define TIMER_FREQ	84000000

	#elif defined(STM32F429_439xx)

	#define TIMER_FREQ	90000000

	#endif


#define	__tb6612_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
											else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
											else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
											else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
											else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
											else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
											else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
										}

#define	__tb6612_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
													GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

#define	__gpio_write_bit(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#endif

#endif

/* tb6612私有数据结构体 */
typedef struct {
	pwm_dev_t pwma;	// PWMA
	pwm_dev_t pwmb;	// PWMB
} tb6612_priv_data_t;	

/* 函数声明 */				
static void __tb6612_motor_control(	tb6612_gpio_port_t in1_port, tb6612_gpio_pin_t in1_pin,
									tb6612_gpio_port_t in2_port, tb6612_gpio_pin_t in2_pin,
									pwm_dev_t *pwm, int8_t speed);					
static int8_t __tb6612_set_speed(tb6612_dev_t *dev, motor_id_t motor_id, int8_t speed);
static int8_t __tb6612_deinit(tb6612_dev_t *dev);

/******************************************************************************
 * @brief	初始化tb6612
 * @param	dev	:	tb6612_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t tb6612_init(tb6612_dev_t *dev)
{
	if (!dev)
		return -1;

	/* 保存私有数据 */
	dev->priv_data = (tb6612_priv_data_t *)malloc(sizeof(tb6612_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	tb6612_priv_data_t *priv_data = (tb6612_priv_data_t *)dev->priv_data;

	if (dev->config.pwma_timx != NULL)
	{
		priv_data->pwma.config.timx = dev->config.pwma_timx;
		priv_data->pwma.config.oc_channel = dev->config.pwma_oc_channel;
		priv_data->pwma.config.arr = 100 - 1;
		priv_data->pwma.config.psc = TIMER_FREQ / 2000000 - 1;
		priv_data->pwma.config.port = dev->config.pwma_port;
		priv_data->pwma.config.pin = dev->config.pwma_pin;

		/* 初始化PWM */
		pwm_init(&priv_data->pwma);

		/* 配置时钟与GPIO */
		__tb6612_io_clock_enable(dev->config.ain1_port);
		__tb6612_io_clock_enable(dev->config.ain2_port);
		__tb6612_config_io_out_pp(dev->config.ain1_port, dev->config.ain1_pin);
		__tb6612_config_io_out_pp(dev->config.ain2_port, dev->config.ain2_pin);
	}
	if (dev->config.pwmb_timx != NULL)
	{
		priv_data->pwmb.config.timx = dev->config.pwmb_timx;
		priv_data->pwmb.config.oc_channel = dev->config.pwmb_oc_channel;
		priv_data->pwmb.config.arr = 100 - 1;
		priv_data->pwmb.config.psc = TIMER_FREQ / 2000000 - 1;
		priv_data->pwmb.config.port = dev->config.pwmb_port;
		priv_data->pwmb.config.pin = dev->config.pwmb_pin;

		/* 初始化PWM */
		pwm_init(&priv_data->pwmb);

		/* 配置时钟与GPIO */
		__tb6612_io_clock_enable(dev->config.bin1_port);
		__tb6612_io_clock_enable(dev->config.bin2_port);
		__tb6612_config_io_out_pp(dev->config.bin1_port, dev->config.bin1_pin);
		__tb6612_config_io_out_pp(dev->config.bin2_port, dev->config.bin2_pin);
	}

	/* 函数指针赋值 */
	dev->set_speed = __tb6612_set_speed;
	dev->deinit = __tb6612_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	TB6612电机控制，内部使用
 * @param	in1_port	:	输入通道1端口
 * @param	in1_pin		:	输入通道1引脚
 * @param	in2_port	:	输入通道2端口
 * @param	in2_pin		:	输入通道2引脚
 * @param	pwm			:	pwm_dev_t 结构体指针
 * @param	speed		:	速度，范围-100~100
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static void __tb6612_motor_control(	tb6612_gpio_port_t in1_port, tb6612_gpio_pin_t in1_pin,
									tb6612_gpio_port_t in2_port, tb6612_gpio_pin_t in2_pin,
									pwm_dev_t *pwm, int8_t speed)
{
	if (speed >= 0)
	{
		/* 正转 */
		__gpio_write_bit(in1_port, in1_pin, GPIO_LEVEL_HIGH);
		__gpio_write_bit(in2_port, in2_pin, GPIO_LEVEL_LOW);
	}
	else
	{
		/* 反转 */
		__gpio_write_bit(in1_port, in1_pin, GPIO_LEVEL_LOW);
		__gpio_write_bit(in2_port, in2_pin, GPIO_LEVEL_HIGH);
		speed = -speed;
	}

	if (speed > 100) speed = 100;

	pwm->set_compare(pwm, speed);
}

/******************************************************************************
 * @brief	TB6612设置速度
 * @param	dev			:	tb6612_dev_t 结构体指针
 * @param	motor_id	:	电机ID，motor_id_t 枚举类型
 * @param	speed		:	速度，范围-100~100
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __tb6612_set_speed(tb6612_dev_t *dev, motor_id_t motor_id, int8_t speed)
{
	if (!dev || !dev->init_flag)
		return -1;

	tb6612_priv_data_t *priv_data = (tb6612_priv_data_t *)dev->priv_data;

	if (motor_id == MOTOR_1)
	{
		__tb6612_motor_control(	dev->config.ain1_port, dev->config.ain1_pin,
								dev->config.ain2_port, dev->config.ain2_pin,
								&priv_data->pwma, speed);
	}
	else if (motor_id == MOTOR_2)
	{
		__tb6612_motor_control(	dev->config.bin1_port, dev->config.bin1_pin,
								dev->config.bin2_port, dev->config.bin2_pin,
								&priv_data->pwmb, speed);
	}
	else
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @brief	去初始化tb6612
 * @param	dev	:	tb6612_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __tb6612_deinit(tb6612_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	tb6612_priv_data_t *priv_data = (tb6612_priv_data_t *)dev->priv_data;

	/* 去初始化PWM */
	if (dev->config.pwma_timx != NULL)
	{
		priv_data->pwma.deinit(&priv_data->pwma);
	}
	if (dev->config.pwmb_timx != NULL)
	{
		priv_data->pwmb.deinit(&priv_data->pwmb);
	}
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
