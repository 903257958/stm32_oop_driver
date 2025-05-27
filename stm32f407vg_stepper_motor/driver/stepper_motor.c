#include "delay.h"
#include "stepper_motor.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__stepper_motor_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
															else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
															else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
															else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
															else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
															else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
															else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
														}

#define	__stepper_motor_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
														GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
														GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
														GPIO_InitStructure.GPIO_Pin = pin ; \
														GPIO_Init(port, &GPIO_InitStructure); \
													}

#define	__stepper_motor_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__stepper_motor_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
															else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
															else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
															else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
															else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
															else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
															else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
														}

#define	__stepper_motor_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
														GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
														GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
														GPIO_InitStructure.GPIO_Pin = pin; \
														GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
														GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
														GPIO_Init(port, &GPIO_InitStructure); \
													}

#define	__stepper_motor_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#endif

/* 步进电机引脚设置数字 */
static uint8_t g_stepper_motor_pin_ctrl[8] = {0x2, 0x3, 0x1, 0x9, 0x8, 0xc, 0x4, 0x6};
static int8_t g_stepper_motor_index = 0;

/* 函数声明 */
static void __stepper_motor_control(StepperMotorDev_t *dev, int cnt, uint16_t time_ms);
static void __stepper_motor_disable(StepperMotorDev_t *dev);
static int8_t __stepper_motor_deinit(StepperMotorDev_t *dev);

/******************************************************************************
 * @brief	初始化步进电机
 * @param	dev	:  StepperMotorDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t stepper_motor_init(StepperMotorDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 配置时钟与GPIO */
	__stepper_motor_config_gpio_clock_enable(dev->config.in1_port);
	__stepper_motor_config_gpio_clock_enable(dev->config.in2_port);
	__stepper_motor_config_gpio_clock_enable(dev->config.in3_port);
	__stepper_motor_config_gpio_clock_enable(dev->config.in4_port);
	__stepper_motor_config_io_out_pp(dev->config.in1_port, dev->config.in1_pin);
	__stepper_motor_config_io_out_pp(dev->config.in2_port, dev->config.in2_pin);
	__stepper_motor_config_io_out_pp(dev->config.in3_port, dev->config.in3_pin);
	__stepper_motor_config_io_out_pp(dev->config.in4_port, dev->config.in4_pin);
	
	/* 函数指针赋值 */
	dev->control = __stepper_motor_control;
	dev->disable = __stepper_motor_disable;
	dev->deinit = __stepper_motor_deinit;
	
	/*默认关闭*/
	__stepper_motor_disable(dev);
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	步进电机引脚操作
 * @param	dev		:  StepperMotorDev_t 结构体指针
 * @param	index	:  步进电机引脚控制索引
 * @return	无
 ******************************************************************************/
static void __stepper_motor_set_pins(StepperMotorDev_t *dev, int index)
{
	if ((g_stepper_motor_pin_ctrl[index] & (1 << 0)))
		__stepper_motor_io_write(dev->config.in1_port, dev->config.in1_pin, GPIO_LEVEL_HIGH);
	else
		__stepper_motor_io_write(dev->config.in1_port, dev->config.in1_pin, GPIO_LEVEL_LOW);

	if ((g_stepper_motor_pin_ctrl[index] & (1 << 1)))
		__stepper_motor_io_write(dev->config.in2_port, dev->config.in2_pin, GPIO_LEVEL_HIGH);
	else
		__stepper_motor_io_write(dev->config.in2_port, dev->config.in2_pin, GPIO_LEVEL_LOW);

	if ((g_stepper_motor_pin_ctrl[index] & (1 << 2)))
		__stepper_motor_io_write(dev->config.in3_port, dev->config.in3_pin, GPIO_LEVEL_HIGH);
	else
		__stepper_motor_io_write(dev->config.in3_port, dev->config.in3_pin, GPIO_LEVEL_LOW);

	if ((g_stepper_motor_pin_ctrl[index] & (1 << 3)))
		__stepper_motor_io_write(dev->config.in4_port, dev->config.in4_pin, GPIO_LEVEL_HIGH);
	else
		__stepper_motor_io_write(dev->config.in4_port, dev->config.in4_pin, GPIO_LEVEL_LOW);

}

/******************************************************************************
 * @brief	控制步进电机
 * @param	dev		:  StepperMotorDev_t 结构体指针
 * @param	cnt		:  步进的次数，大于0表示顺时针步进，小于0表示逆时针步进，4096为旋转一圈
 * @param	time_ms	:  每次步进的时间间隔（单位：毫秒）
 * @return	无
 ******************************************************************************/
static void __stepper_motor_control(StepperMotorDev_t *dev, int cnt, uint16_t time_ms)
{
	int step;

	if (cnt >= 0)
	{
		/* 顺时针步进 */
		for (step = 0; step < cnt; step++)
		{
			__stepper_motor_set_pins(dev, g_stepper_motor_index);
			delay_ms(time_ms);
			g_stepper_motor_index--;
			if (g_stepper_motor_index == -1)
			{
				g_stepper_motor_index = 7;
			}
		}
	}
	else
	{
		/* 逆时针步进 */
		cnt = -cnt;
		for (step = 0; step < cnt; step++)
		{
			__stepper_motor_set_pins(dev, g_stepper_motor_index);
			delay_ms(time_ms);
			g_stepper_motor_index++;
			if (g_stepper_motor_index == 8)
			{
				g_stepper_motor_index = 0;
			}
		}
	}

	/* 旋转到位后关闭步进电机，不再消耗电源 */
	__stepper_motor_disable(dev);
}

/******************************************************************************
 * @brief	关闭步进电机
 * @param	dev   :  StepperMotorDev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __stepper_motor_disable(StepperMotorDev_t *dev)
{
	/* gpio的电平都为0，对应控制线均为高阻状态，不导通 */
	__stepper_motor_io_write(dev->config.in1_port, dev->config.in1_pin, GPIO_LEVEL_LOW);
	__stepper_motor_io_write(dev->config.in2_port, dev->config.in2_pin, GPIO_LEVEL_LOW);
	__stepper_motor_io_write(dev->config.in3_port, dev->config.in3_pin, GPIO_LEVEL_LOW);
	__stepper_motor_io_write(dev->config.in4_port, dev->config.in4_pin, GPIO_LEVEL_LOW);
}

/******************************************************************************
 * @brief	去初始化步进电机
 * @param	dev   :  StepperMotorDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __stepper_motor_deinit(StepperMotorDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
