#include <stdlib.h>
#include <stdio.h>
#include "led.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__led_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{led_log("gpio clock no enable\r\n");} \
												}

#define	__led_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__led_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#elif defined(STM32F40_41xxx)

#define	__led_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{led_log("gpio clock no enable\r\n");} \
												}

#define	__led_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__led_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#endif

/*LED私有数据结构体*/
typedef struct {
	bool status;		//状态，false灭/true亮，默认灭
}LEDPrivData_t;

static int __led_on(LEDDev_t *pDev);
static int __led_off(LEDDev_t *pDev);
static int __led_get_status(LEDDev_t *pDev);
static int __led_toggle(LEDDev_t *pDev);
static int __led_deinit(LEDDev_t *pDev);

/******************************************************************************
 * @brief	初始化LED
 * @param	pDev	:  LEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int led_init(LEDDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/*保存私有数据*/
	pDev->pPrivData = (LEDPrivData_t *)malloc(sizeof(LEDPrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	LEDPrivData_t *pPrivData = (LEDPrivData_t *)pDev->pPrivData;
	
	/*配置时钟与GPIO*/	
	__led_config_gpio_clock_enable(pDev->info.port);
	__led_config_io_out_pp(pDev->info.port, pDev->info.pin);
	
	/*函数指针赋值*/
	pDev->on = __led_on;
	pDev->off = __led_off;
	pDev->get_status = __led_get_status;
	pDev->toggle = __led_toggle;
	pDev->deinit = __led_deinit;
	
	/*默认关闭*/
	pPrivData->status = false;
	__led_off(pDev);
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	打开LED
 * @param	pDev   :  LEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __led_on(LEDDev_t *pDev)
{
	LEDPrivData_t *pPrivData = (LEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	 __led_io_write(pDev->info.port, pDev->info.pin, !pDev->info.offLevel);	//LED亮
	pPrivData->status = true;												//保存此时LED的状态
	
	return 0;
}

/******************************************************************************
 * @brief	关闭LED
 * @param	pDev   :  LEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __led_off(LEDDev_t *pDev)
{
	LEDPrivData_t *pPrivData = (LEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	 __led_io_write(pDev->info.port, pDev->info.pin, pDev->info.offLevel);	//LED灭
	pPrivData->status = false;												//保存此时LED的状态
	
	return 0;
}

/******************************************************************************
 * @brief	获取LED的状态
 * @param	pDev   :  LEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __led_get_status(LEDDev_t *pDev)
{
	LEDPrivData_t *pPrivData = (LEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	return pPrivData->status;
}

/******************************************************************************
 * @brief	翻转LED
 * @param	pDev   :  LEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __led_toggle(LEDDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	if(pDev->get_status(pDev))
	{
		pDev->off(pDev);
	}
	else
	{
		pDev->on(pDev);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	去初始化LED
 * @param	pDev   :  LEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __led_deinit(LEDDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	/*释放私有数据内存*/
	free(pDev->pPrivData);
    pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	//修改初始化标志
	
	return 0;
}
