#include "ws2812b.h"

#if defined(STM32F10X_MD)

#define TIMER_FREQ	72000000

#define	__ws2812b_dma_clock_enable(timx)	{	if (timx == TIM2)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if (timx == TIM3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if (timx == TIM4)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
											}

#define __ws2812b_get_dma_channel(timx)	(	timx == TIM2 ? DMA1_Channel2 : \
											timx == TIM3 ? DMA1_Channel3 : \
											timx == TIM4 ? DMA1_Channel7 : \
											(int)0	)

#define __ws2812b_get_dma_base(channel)	(	channel == 1 ? TIM_DMABase_CCR1 : \
											channel == 2 ? TIM_DMABase_CCR2 : \
											channel == 3 ? TIM_DMABase_CCR3 : \
											channel == 4 ? TIM_DMABase_CCR4 : \
											(int)0	)

#define __ws2812b_get_dma_tc_flag(timx) (	timx == TIM2 ? DMA1_FLAG_TC2 : \
											timx == TIM3 ? DMA1_FLAG_TC3 : \
											timx == TIM4 ? DMA1_FLAG_TC7 : \
											(int)0)

#endif

/* WS2812B设备指针数组 */
ws2812b_dev_t *g_ws2812b_dev[WS2812B_MAX_DEVICE_NUM];
uint8_t g_ws2812b_num;

/* WS2812B私有数据结构体 */
typedef struct {
	pwm_dev_t pwm;
	uint8_t index;
} ws2812b_priv_data_t;

/* 储存转换后的PWM数据 */
uint16_t g_ws2812b_pwm_buf[WS2812B_MAX_DEVICE_NUM][24 * WS2812B_MAX_LED_NUM + 1]; 

/* 内部使用 */
static void __ws2812b_dma_init(ws2812b_dev_t *dev);
static void __ws2812b_dma_start(ws2812b_dev_t *dev, uint16_t data_len);
static bool __ws2812b_dma_transfer_complete(ws2812b_dev_t *dev);
static uint32_t rgb_to_grb(uint32_t rgb);

/* 外部接口 */
static int8_t __ws2812b_off(ws2812b_dev_t *dev);
static int8_t __ws2812b_set_color(ws2812b_dev_t *dev, uint32_t *color_rgb_buf);
static int8_t __ws2812b_set_single_color(ws2812b_dev_t *dev, uint32_t color_rgb);
static int8_t __ws2812b_deinit(ws2812b_dev_t *dev);

/******************************************************************************
 * @brief	初始化WS2812B
 * @param	dev	:	ws2812b_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t ws2812b_init(ws2812b_dev_t *dev)
{
	if (!dev)
		return -1;

	if (g_ws2812b_num >= WS2812B_MAX_LED_NUM)
		return -2;	// 初始化前发现已注册的设备已经等于最大设备数，初始化失败

	/* 保存私有数据 */
	dev->priv_data = (ws2812b_priv_data_t *)malloc(sizeof(ws2812b_priv_data_t));
	if (!dev->priv_data)
		return -3;
	
	ws2812b_priv_data_t *priv_data = (ws2812b_priv_data_t *)dev->priv_data;

	priv_data->index = g_ws2812b_num;

	/* 记录设备指针用于中断处理 */
	g_ws2812b_dev[priv_data->index] = dev;

	/* WS2812B设备数加1 */
	g_ws2812b_num++;

	priv_data->pwm.config.timx = dev->config.timx;
	priv_data->pwm.config.oc_channel = dev->config.oc_channel;
	priv_data->pwm.config.arr = (TIMER_FREQ / 800000) - 1;	// PWM频率为800KHz
	priv_data->pwm.config.psc = 0;
	priv_data->pwm.config.port = dev->config.port;
	priv_data->pwm.config.pin = dev->config.pin;

	/* 初始化PWM */
	pwm_init(&priv_data->pwm);

    /* DMA配置 */
	__ws2812b_dma_init(dev);
	
	/* 定时器DMA配置与使能 */
	TIM_DMAConfig(dev->config.timx, __ws2812b_get_dma_base(dev->config.oc_channel), TIM_DMABurstLength_1Transfer);
	TIM_DMACmd(dev->config.timx, TIM_DMA_Update, ENABLE);

	/* 函数指针赋值 */
	dev->set_color = __ws2812b_set_color;
	dev->set_single_color = __ws2812b_set_single_color;
	dev->off = __ws2812b_off;
	dev->deinit = __ws2812b_deinit;

	dev->init_flag = true;
	
	return 0;
}

/******************************************************************************
 * @brief	WS2812B DMA初始化
 * @param	dev	:	ws2812b_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __ws2812b_dma_init(ws2812b_dev_t *dev)
{
	ws2812b_priv_data_t *priv_data = (ws2812b_priv_data_t *)dev->priv_data;
	dma_channel_t dma_channel = __ws2812b_get_dma_channel(dev->config.timx);

	__ws2812b_dma_clock_enable(dev->config.timx);

	DMA_DeInit(dma_channel);
	
	DMA_InitTypeDef DMA_InitStructure;
	switch (dev->config.oc_channel)
	{
		case 1:	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&dev->config.timx->CCR1);	break;
		case 2:	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&dev->config.timx->CCR2);	break;
		case 3:	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&dev->config.timx->CCR3);	break;
		case 4:	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&dev->config.timx->CCR4);	break;
	}
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_ws2812b_pwm_buf[priv_data->index];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(dma_channel, &DMA_InitStructure);
	
	DMA_Cmd(dma_channel, ENABLE);
}

/******************************************************************************
 * @brief	WS2812B开启一次DMA传输
 * @param	dev			:	ws2812b_dev_t 结构体指针
 * @param	data_len	:	传输数据量
 * @return	无
 ******************************************************************************/
static void __ws2812b_dma_start(ws2812b_dev_t *dev, uint16_t data_len)
{
	dma_channel_t dma_channel = __ws2812b_get_dma_channel(dev->config.timx);

	DMA_Cmd(dma_channel, DISABLE);
	DMA_SetCurrDataCounter(dma_channel, data_len);
	DMA_Cmd(dma_channel, ENABLE);
}

/******************************************************************************
 * @brief   检查DMA传输是否完成
 * @param   dev     :   ws2812b_dev_t 结构体指针
 * @return  true: 完成, false: 未完成
 ******************************************************************************/
static bool __ws2812b_dma_transfer_complete(ws2812b_dev_t *dev)
{
    if (DMA_GetFlagStatus(__ws2812b_get_dma_tc_flag(dev->config.timx)) != RESET)
	{
        DMA_ClearFlag(__ws2812b_get_dma_tc_flag(dev->config.timx));
        return true;
    }
    return false;
}

/******************************************************************************
 * @brief	RGB转GRB，格式：0xRRGGBB -> 0xGGRRBB
 * @param	rgb	:	RGB颜色值
 * @return	GRB颜色值
 ******************************************************************************/
static uint32_t rgb_to_grb(uint32_t rgb)
{
    uint8_t r = (rgb >> 16) & 0xFF;  			// 提取红色
    uint8_t g = (rgb >> 8) & 0xFF;   			// 提取绿色
    uint8_t b = rgb & 0xFF;          			// 提取蓝色

    uint32_t grb = (g << 16) | (r << 8) | b;	// GGRRBB

    return grb;
}

/******************************************************************************
 * @brief	WS2812B设置颜色
 * @param	dev				:	ws2812b_dev_t 结构体指针
 * @param	color_rgb_buf	:	RGB颜色数组，格式为0xRRGGBB
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ws2812b_set_color(ws2812b_dev_t *dev, uint32_t *color_rgb_buf)
{
	if (!dev || !dev->init_flag)
		return -1;

	ws2812b_priv_data_t *priv_data = (ws2812b_priv_data_t *)dev->priv_data;
	
	uint8_t i, j;
	uint32_t color_grb_buf[WS2812B_MAX_LED_NUM];

	for (i = 0; i < dev->config.led_num; i++)
	{
		color_grb_buf[i] = rgb_to_grb(color_rgb_buf[i]);
	}

	for (j = 0; j < dev->config.led_num; j++)
	{
		for (i = 0; i < 24; i++)
		{
			if (color_grb_buf[j] & (0x800000 >> i))
			{
				g_ws2812b_pwm_buf[priv_data->index][j * 24 + i + 1] = 60;
			}
			else
			{
				g_ws2812b_pwm_buf[priv_data->index][j * 24 + i + 1] = 30;
			}
		}
	}

	/* 开启一次DMA传输 */
	__ws2812b_dma_start(dev, 24 * dev->config.led_num + 1);
	TIM_Cmd(dev->config.timx, ENABLE);

	/* 等待DMA传输完成 */
    while (!__ws2812b_dma_transfer_complete(dev));

	/* 传输完成后停止定时器 */
    TIM_Cmd(dev->config.timx, DISABLE);
    
    /* 设置占空比为0（复位信号） */
    switch (dev->config.oc_channel)
	{
        case 1: TIM_SetCompare1(dev->config.timx, 0); break;
        case 2: TIM_SetCompare2(dev->config.timx, 0); break;
        case 3: TIM_SetCompare3(dev->config.timx, 0); break;
        case 4: TIM_SetCompare4(dev->config.timx, 0); break;
    }

	return 0;
}

/******************************************************************************
 * @brief	WS2812B设置单种颜色
 * @param	dev			:	ws2812b_dev_t 结构体指针
 * @param	color_rgb	:	RGB颜色，格式为0xRRGGBB
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ws2812b_set_single_color(ws2812b_dev_t *dev, uint32_t color_rgb)
{
	if (!dev || !dev->init_flag)
		return -1;

	uint32_t color_rgb_buf[WS2812B_MAX_LED_NUM];
	uint8_t i;

	for (i = 0; i < dev->config.led_num; i++)
    {
        color_rgb_buf[i] = color_rgb;
    }

	__ws2812b_set_color(dev, color_rgb_buf);

	return 0;
}

/******************************************************************************
 * @brief	WS2812B熄灭
 * @param	dev	:	ws2812b_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ws2812b_off(ws2812b_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	__ws2812b_set_single_color(dev, 0x000000);

	return 0;
}

/******************************************************************************
 * @brief	去初始化WS2812B
 * @param	dev	:	ws2812b_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ws2812b_deinit(ws2812b_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	ws2812b_priv_data_t *priv_data = (ws2812b_priv_data_t *)dev->priv_data;

	/* 去初始化PWM */
	priv_data->pwm.deinit(&priv_data->pwm);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
