#include "key.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__key_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{Key_Log("key clock no enable\r\n");} \
												}

#define	__key_config_io_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__key_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __key_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#elif defined(STM32F40_41xxx)

#define	__key_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{Key_Log("gpio clock no enable\r\n");} \
												}

#define	__key_config_io_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__key_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __key_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#endif
									
static bool __key_is_press(KeyDev_t *pDev);
static int __key_deinit(KeyDev_t *pDev);
										
/******************************************************************************
 * @brief	初始化按键
 * @param	pDev		:  KeyDev_t结构体指针
 * @param	port		:  端口
 * @param	pin			:  引脚
 * @param	pressLevel	:  按键按下的时候IO口的电平
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int key_init(KeyDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/*配置时钟与GPIO*/
	__key_config_gpio_clock_enable(pDev->info.port);
	
	if(pDev->info.pressLevel == GPIO_LEVEL_HIGH)			//根据pressLevel配置为上拉或下拉输入
	{
		__key_config_io_in_pd(pDev->info.port, pDev->info.pin);
	}
	else
	{
		__key_config_io_in_pu(pDev->info.port, pDev->info.pin);
	}
	
	/*函数指针赋值*/
	pDev->is_press = __key_is_press;
	pDev->deinit = __key_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	判断按键是否被按下
 * @param	pDev   :  KeyDev_t结构体指针
 * @return	true, 表示按键被按下； false，表示按键未被按下。
 ******************************************************************************/
static bool __key_is_press(KeyDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	if (__key_io_read(pDev->info.port, pDev->info.pin) == pDev->info.pressLevel)
	{
		while((__key_io_read(pDev->info.port, pDev->info.pin) == pDev->info.pressLevel));
		return true;
	}
	
	return false;
}

/******************************************************************************
 * @brief	去初始化按键
 * @param	pDev   :  KeyDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __key_deinit(KeyDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	pDev->initFlag = false;	//修改初始化标志
	
	return 0;
}
