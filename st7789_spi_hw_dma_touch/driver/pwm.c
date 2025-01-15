#include "PWM.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define	__pwm_config_timer_clock_enable(TIMx)	{	if(TIMx == TIM2)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
													else if(TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
													else if(TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
													else if(TIMx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
													else					{pwm_log("pwm timer clock no enable\r\n");} \
												}

#define	__pwm_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{pwm_log("pwm gpio clock no enable\r\n");} \
												}

#define	__pwm_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#elif defined(STM32F40_41xxx) || defined(STM32F411xE)

#define	__pwm_config_timer_clock_enable(TIMx)	{	if(TIMx == TIM2)		{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);} \
													else if(TIMx == TIM3)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);} \
													else if(TIMx == TIM4)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);} \
													else if(TIMx == TIM5)	{RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);} \
													else					{pwm_log("pwm timer clock no enable\r\n");} \
												}

#define	__pwm_config_gpio_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{pwm_log("pwm gpio clock no enable\r\n");} \
												}

#define	__pwm_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __pwm_get_gpio_pin_source(pin)	(	pin == GPIO_Pin_0 ? GPIO_PinSource0 : \
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
											(int)0	)

#define __pwm_get_gpio_af(TIMx)	(	TIMx == TIM2 ? GPIO_AF_TIM2 : \
									TIMx == TIM3 ? GPIO_AF_TIM3 : \
									TIMx == TIM4 ? GPIO_AF_TIM4 : \
									TIMx == TIM5 ? GPIO_AF_TIM5 : \
									(int)0	)

#endif

/* 函数声明 */									
static void __pwm_set_psc(PWMDev_t *pDev, uint16_t psc);
static void __pwm_set_arr(PWMDev_t *pDev, uint16_t arr);
static void __pwm_set_compare(PWMDev_t *pDev, uint16_t compare);
static int __pwm_deinit(PWMDev_t *pDev);

/******************************************************************************
 * @brief	初始化PWM
			主频72MHz，72M/PSC为计数频率，其倒数为计数周期
			用作微秒级定时器时，PSC = 72 - 1，计数周期 = 1us，定时周期 = (ASC + 1)(us)，最大定时周期约为65.5ms
			用作毫秒级定时器时，PSC = 7200 - 1，计数周期 = 0.1ms，定时周期 = ((ASC + 1)/10))(ms)，最大定时周期约为6.55s
 * @param	pDev	:	PWMDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int pwm_init(PWMDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 配置时钟与GPIO */
	__pwm_config_timer_clock_enable(pDev->info.timx);
	__pwm_config_gpio_clock_enable(pDev->info.port);
	
	__pwm_config_io_af_pp(pDev->info.port, pDev->info.pin);					// 复用推挽输出

	#if defined(STM32F40_41xxx) || defined(STM32F411xE)
	GPIO_PinAFConfig(pDev->info.port, __pwm_get_gpio_pin_source(pDev->info.pin), __pwm_get_gpio_af(pDev->info.timx));
	#endif
	
	/* 配置时钟源 */
	TIM_InternalClockConfig(pDev->info.timx);
	
	/* 时基单元初始化 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;				// 时钟分频参数，不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;			// 计数器模式：向上计数
	TIM_TimeBaseInitStructure.TIM_Prescaler = pDev->info.psc;				// PSC预分频器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_Period = pDev->info.arr;					// ARR自动重装器的值，范围0~65535
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;					// 重复计数器的值（高级定时器用）
	TIM_TimeBaseInit(pDev->info.timx, &TIM_TimeBaseInitStructure);
	
	/* 输出比较配置 */
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);									// 给结构体赋默认值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;						// 设置输出比较模式：PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;				// 设置输出比较极性：高极性（REF极性不反转）
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;			// 设置输出使能
	TIM_OCInitStructure.TIM_Pulse = 1;		// 设置CCR	Freq=CK_PSC/(PSC+1)/(ARR+1)	Duty=CCR/(ARR+1)	Reso=1/(ARR+1)
	if (pDev->info.OCChannel == 1)
	{
		TIM_OC1Init(pDev->info.timx, &TIM_OCInitStructure);
	}
	else if (pDev->info.OCChannel == 2)
	{
		TIM_OC2Init(pDev->info.timx, &TIM_OCInitStructure);
	}
	else if (pDev->info.OCChannel == 3)
	{
		TIM_OC3Init(pDev->info.timx, &TIM_OCInitStructure);
	}
	else if (pDev->info.OCChannel == 4)
	{
		TIM_OC4Init(pDev->info.timx, &TIM_OCInitStructure);
	}
	
	/* 启用定时器 */
	TIM_Cmd(pDev->info.timx, ENABLE);
	
	/* 函数指针赋值 */
	pDev->set_psc = __pwm_set_psc;
	pDev->set_arr = __pwm_set_arr;
	pDev->set_compare = __pwm_set_compare;
	pDev->deinit = __pwm_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	PWM设置PSC的值
 * @param	pDev	：	PWMDev_t结构体指针
 * @param	psc		：	要写入的PSC的值
 * @return	无
 ******************************************************************************/
static void __pwm_set_psc(PWMDev_t *pDev, uint16_t psc)
{
    TIM_Cmd(pDev->info.timx, DISABLE);
    pDev->info.timx->PSC = psc;
    pDev->info.timx->EGR = TIM_PSCReloadMode_Update;	 // 重新加载预分频器值
    TIM_Cmd(pDev->info.timx, ENABLE);
}

/******************************************************************************
 * @brief	PWM设置ARR的值
 * @param	pDev	：	PWMDev_t结构体指针
 * @param	arr		：	要写入的ARR的值
 * @return	无
 ******************************************************************************/
static void __pwm_set_arr(PWMDev_t *pDev, uint16_t arr)
{
    TIM_Cmd(pDev->info.timx, DISABLE);
	pDev->info.timx->ARR = arr;
    pDev->info.timx->EGR = TIM_PSCReloadMode_Update;	 // 重新加载预分频器值
    TIM_Cmd(pDev->info.timx, ENABLE);
}

/******************************************************************************
 * @brief	PWM设置CCR的值
 * @param	pDev	：	PWMDev_t结构体指针
 * @param	compare	：	要写入的CCR的值
 * @return	无
 ******************************************************************************/
static void __pwm_set_compare(PWMDev_t *pDev, uint16_t compare)
{
	if(pDev->info.OCChannel == 1)
	{
		TIM_SetCompare1(pDev->info.timx, compare);
	}
	else if(pDev->info.OCChannel == 2)
	{
		TIM_SetCompare2(pDev->info.timx, compare);
	}
	else if(pDev->info.OCChannel == 3)
	{
		TIM_SetCompare3(pDev->info.timx, compare);
	}
	else if(pDev->info.OCChannel == 4)
	{
		TIM_SetCompare4(pDev->info.timx, compare);
	}
}

/******************************************************************************
 * @brief	去初始化PWM
 * @param	pDev   :  PWMDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __pwm_deinit(PWMDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	pDev->initFlag = false;	// 修改初始化标志
	
	return 0;
}
