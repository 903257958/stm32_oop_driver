#include "adc.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define	__adc_config_clock_enable(adcx)	{	if(adcx == ADC1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);} \
											else if(adcx == ADC2)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);} \
											else if(adcx == ADC3)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);} \
										}

#define	__adc_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												}

#define	__adc_config_io_in_an(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__adc_config_clock_enable(adcx)	{	if(adcx == ADC1)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);} \
											else if(adcx == ADC2)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);} \
											else if(adcx == ADC3)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);} \
										}

#define	__adc_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}

#define	__adc_config_io_in_an(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#endif

static int8_t __adc_get_val(ADCDev_t *dev, uint16_t *val);
static int8_t __adc_deinit(ADCDev_t *dev);

/******************************************************************************
 * @brief	初始化ADC
 * @param	dev	:  ADCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t adc_init(ADCDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 配置时钟与GPIO */	
	__adc_config_gpio_clock_enable(dev->config.port);
	__adc_config_io_in_an(dev->config.port, dev->config.pin);
	
	/* 配置ADC */
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	/* 开启ADC时钟 */
	__adc_config_clock_enable(dev->config.adcx);

	/* 设置ADC时钟，最大时钟配置为36MHz */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);	// 选择时钟6分频，ADCCLK = 72MHz / 6 = 12MHz

	/* ADC初始化 */
	ADC_InitTypeDef ADC_InitStructure;									// 定义结构体变量
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					// 模式，选择独立模式，即单独使用ADCx
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				// 数据对齐，选择右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	// 外部触发，使用软件触发，不需要外部触发
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					// 连续转换，失能，每转换一次规则组序列后停止
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;						// 扫描模式，失能，只转换规则组的序列1这一个位置
	ADC_InitStructure.ADC_NbrOfChannel = 1;								// 通道数，为1，仅在扫描模式下，才需要指定大于1的数，在非扫描模式下，只能是1
	ADC_Init(dev->config.adcx, &ADC_InitStructure);						// 将结构体变量交给ADC_Init，配置ADCx
	
	/* ADC使能 */
	ADC_Cmd(dev->config.adcx, ENABLE);									// 使能ADCx，ADC开始运行

	/* ADC校准 */
	ADC_ResetCalibration(dev->config.adcx);								// 固定流程，内部有电路会自动执行校准
	while (ADC_GetResetCalibrationStatus(dev->config.adcx) == SET);
	ADC_StartCalibration(dev->config.adcx);
	while (ADC_GetCalibrationStatus(dev->config.adcx) == SET);

	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	/* 开启ADC时钟 */
	__adc_config_clock_enable(dev->config.adcx);

	/* ADC通用初始化 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;						// 独立模式
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;						// 时钟4分频，84MHz / 4 = 21MHz (最大时钟配置为36MHz)
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;			// DMA模式，仅适用于双重或三重交错模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;	// 两个采样阶段间的延迟
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC初始化 */
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	// 12位分辨率
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;			// 不使用扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		// 不使用连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;	// 不使用外部触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	// 数据右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 1;				// 通道数为1
	ADC_Init(dev->config.adcx, &ADC_InitStructure);

	/* ADC使能 */
	ADC_Cmd(dev->config.adcx, ENABLE);						// 使能ADCx，ADC开始运行

	#endif

	/* 函数指针赋值 */
	dev->get_val = __adc_get_val;
	dev->deinit = __adc_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	获取ADC转换值
 * @param	dev   	:  ADCDev_t 结构体指针
 * @param	val   	:  ADC转换值
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __adc_get_val(ADCDev_t *dev, uint16_t *val)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	ADC_RegularChannelConfig(dev->config.adcx, dev->config.channel, 1, ADC_SampleTime_55Cycles5);	// 在每次转换前，根据函数形参灵活更改规则组的通道1
	ADC_SoftwareStartConvCmd(dev->config.adcx, ENABLE);					// 软件触发AD转换一次
	while (ADC_GetFlagStatus(dev->config.adcx, ADC_FLAG_EOC) == RESET);	// 等待EOC标志位，即等待AD转换结束
	*val = ADC_GetConversionValue(dev->config.adcx);					// 读数据寄存器，得到AD转换的结果

	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	ADC_RegularChannelConfig(dev->config.adcx, dev->config.channel, 1, ADC_SampleTime_480Cycles);
	ADC_SoftwareStartConv(dev->config.adcx); 							// 使能指定的ADC1的软件转换启动功能
	while (ADC_GetFlagStatus(dev->config.adcx, ADC_FLAG_EOC) == RESET);	// 等待EOC标志位，即等待AD转换结束
	*val = ADC_GetConversionValue(dev->config.adcx);					// 读数据寄存器，得到AD转换的结果
	#endif
	
	return 0;
}

/******************************************************************************
 * @brief	去初始化ADC
 * @param	dev   :  ADCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __adc_deinit(ADCDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
