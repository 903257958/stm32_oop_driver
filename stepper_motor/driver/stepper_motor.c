#include "stepper_motor.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__stepper_motor_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
															else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
															else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
															else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
															else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
															else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
															else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
															else					{stepper_motor_log("stepper motor gpio clock no enable\r\n");} \
														}

#define	__stepper_motor_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
														GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
														GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
														GPIO_InitStructure.GPIO_Pin = pin ; \
														GPIO_Init(port, &GPIO_InitStructure); \
													}

#define	__stepper_motor_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

	#if !FREERTOS
	static void __stepper_motor_delay_ms(uint32_t ms)
	{
		while(ms--)
		{
			SysTick->LOAD = 72 * 1000;				// 设置定时器重装值
			SysTick->VAL = 0x00;					// 清空当前计数值
			SysTick->CTRL = 0x00000005;				// 设置时钟源为HCLK，启动定时器
			while(!(SysTick->CTRL & 0x00010000));	// 等待计数到0
			SysTick->CTRL = 0x00000004;				// 关闭定时器
		}
	}
	#else
	static void __stepper_motor_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

#elif defined(STM32F40_41xxx)

#define	__stepper_motor_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
															else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
															else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
															else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
															else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
															else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
															else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
															else					{stepper_motor_log("stepper motor gpio clock no enable\r\n");} \
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

	#if !FREERTOS
	static void __stepper_motor_delay_ms(uint32_t ms)
	{
		while(ms--)
		{
			uint32_t temp;	    	 
			SysTick->LOAD = 1000 * 21; 					// 时间加载	 
			SysTick->VAL = 0x00;        				// 清空计数器
			SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ; 	// 开始倒数
			do
			{
				temp = SysTick->CTRL;
			}while((temp & 0x01) && !(temp & (1<<16)));	// 等待时间到达   
			SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 	// 关闭计数器
			SysTick->VAL = 0X00;       					// 清空计数器
		}
	}
	#else		
	static void __stepper_motor_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

#endif

/* 步进电机引脚设置数字 */
static uint8_t gStepperMotorPinCtrl[8] = {0x2, 0x3, 0x1, 0x9, 0x8, 0xc, 0x4, 0x6};
static int8_t gStepperMotorIndex = 0;

/* 函数声明 */
static void __stepper_motor_control(StepperMotorDev_t *pDev, int count, int delayTime);
static void __stepper_motor_disable(StepperMotorDev_t *pDev);
static int __stepper_motor_deinit(StepperMotorDev_t *pDev);

/******************************************************************************
 * @brief	初始化步进电机
 * @param	pDev	:  StepperMotorDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int stepper_motor_init(StepperMotorDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 配置时钟与GPIO */
	__stepper_motor_config_gpio_clock_enable(pDev->info.in1Port);
	__stepper_motor_config_gpio_clock_enable(pDev->info.in2Port);
	__stepper_motor_config_gpio_clock_enable(pDev->info.in3Port);
	__stepper_motor_config_gpio_clock_enable(pDev->info.in4Port);
	__stepper_motor_config_io_out_pp(pDev->info.in1Port, pDev->info.in1Pin);
	__stepper_motor_config_io_out_pp(pDev->info.in2Port, pDev->info.in2Pin);
	__stepper_motor_config_io_out_pp(pDev->info.in3Port, pDev->info.in3Pin);
	__stepper_motor_config_io_out_pp(pDev->info.in4Port, pDev->info.in4Pin);
	
	/* 函数指针赋值 */
	pDev->control = __stepper_motor_control;
	pDev->disable = __stepper_motor_disable;
	pDev->deinit = __stepper_motor_deinit;
	
	/*默认关闭*/
	__stepper_motor_disable(pDev);
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	步进电机引脚操作
 * @param	pDev	:  StepperMotorDev_t结构体指针
 * @param	index	:  步进电机引脚控制索引
 * @return	无
 ******************************************************************************/
static void __stepper_motor_set_pins(StepperMotorDev_t *pDev, int index)
{
	if ((gStepperMotorPinCtrl[index] & (1 << 0)))
		__stepper_motor_io_write(pDev->info.in1Port, pDev->info.in1Pin, GPIO_LEVEL_HIGH);
	else
		__stepper_motor_io_write(pDev->info.in1Port, pDev->info.in1Pin, GPIO_LEVEL_LOW);

	if ((gStepperMotorPinCtrl[index] & (1 << 1)))
		__stepper_motor_io_write(pDev->info.in2Port, pDev->info.in2Pin, GPIO_LEVEL_HIGH);
	else
		__stepper_motor_io_write(pDev->info.in2Port, pDev->info.in2Pin, GPIO_LEVEL_LOW);

	if ((gStepperMotorPinCtrl[index] & (1 << 2)))
		__stepper_motor_io_write(pDev->info.in3Port, pDev->info.in3Pin, GPIO_LEVEL_HIGH);
	else
		__stepper_motor_io_write(pDev->info.in3Port, pDev->info.in3Pin, GPIO_LEVEL_LOW);

	if ((gStepperMotorPinCtrl[index] & (1 << 3)))
		__stepper_motor_io_write(pDev->info.in4Port, pDev->info.in4Pin, GPIO_LEVEL_HIGH);
	else
		__stepper_motor_io_write(pDev->info.in4Port, pDev->info.in4Pin, GPIO_LEVEL_LOW);

}

/******************************************************************************
 * @brief	控制步进电机
 * @param	pDev	:  StepperMotorDev_t结构体指针
 * @param	count	:  步进的次数，大于0表示顺时针步进，小于0表示逆时针步进，4096为旋转一圈
 * @param	timeMs	:  每次步进的时间间隔（单位：毫秒）
 * @return	无
 ******************************************************************************/
static void __stepper_motor_control(StepperMotorDev_t *pDev, int count, int timeMs)
{
	int step;

	if (count >= 0)
	{
		/* 顺时针步进 */
		for (step = 0; step < count; step++)
		{
			__stepper_motor_set_pins(pDev, gStepperMotorIndex);
			__stepper_motor_delay_ms(timeMs);
			gStepperMotorIndex--;
			if (gStepperMotorIndex == -1)
			{
				gStepperMotorIndex = 7;
			}
		}
	}
	else
	{
		/* 逆时针步进 */
		count = -count;
		for (step = 0; step < count; step++)
		{
			__stepper_motor_set_pins(pDev, gStepperMotorIndex);
			__stepper_motor_delay_ms(timeMs);
			gStepperMotorIndex++;
			if (gStepperMotorIndex == 8)
			{
				gStepperMotorIndex = 0;
			}
		}
	}

	/* 旋转到位后关闭步进电机，不再消耗电源 */
	__stepper_motor_disable(pDev);
}

/******************************************************************************
 * @brief	关闭步进电机
 * @param	pDev   :  StepperMotorDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __stepper_motor_disable(StepperMotorDev_t *pDev)
{
	/* gpio的电平都为0，对应控制线均为高阻状态，不导通 */
	__stepper_motor_io_write(pDev->info.in1Port, pDev->info.in1Pin, GPIO_LEVEL_LOW);
	__stepper_motor_io_write(pDev->info.in2Port, pDev->info.in2Pin, GPIO_LEVEL_LOW);
	__stepper_motor_io_write(pDev->info.in3Port, pDev->info.in3Pin, GPIO_LEVEL_LOW);
	__stepper_motor_io_write(pDev->info.in4Port, pDev->info.in4Pin, GPIO_LEVEL_LOW);
}

/******************************************************************************
 * @brief	去初始化步进电机
 * @param	pDev   :  StepperMotorDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __stepper_motor_deinit(StepperMotorDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	pDev->initFlag = false;	// 修改初始化标志
	
	return 0;
}
