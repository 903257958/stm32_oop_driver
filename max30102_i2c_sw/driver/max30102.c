#include "delay.h"
#include "max30102.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__max30102_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
														else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
														else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
														else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
														else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
														else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
														else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
														else					{max30102_log("max30102 gpio clock no enable\r\n");} \
													}
													
#define	__max30102_config_io_in_up(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_InitStructure.GPIO_Pin = pin ; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}
												
#define __max30102_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE)

#define	__max30102_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
														else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
														else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
														else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
														else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
														else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
														else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
														else					{max30102_log("max30102 gpio clock no enable\r\n");} \
													}
													
#define	__max30102_config_io_in_up(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}
												
#define __max30102_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#endif

/* MAX30102私有数据结构体 */
typedef struct {
	I2CDev_t i2c;	// 软件I2C设备
}MAX30102PrivData_t;

/* 函数声明 */
static int __max30102_read_fifo(MAX30102Dev_t *dev, uint32_t *red_led_data, uint32_t *ir_led_data);
static int __max30102_get_data(MAX30102Dev_t *dev);
static int __max30102_deinit(MAX30102Dev_t *dev);

/*****************************MAX30102算法部分**********************************/
#define FS 100
#define BUFFER_SIZE  (FS* 5) 
#define HR_FIFO_SIZE 7
#define MA4_SIZE  4 		// DO NOT CHANGE
#define HAMMING_SIZE  5		// DO NOT CHANGE
#define MAX_BRIGHTNESS 255
#define min(x,y) ((x) < (y) ? (x) : (y))

static uint32_t aun_ir_buffer[500];		//IR LED   红外光数据，用于计算血氧
static uint32_t aun_red_buffer[500];	//Red LED	红光数据，用于计算心率曲线以及计算心率
static int32_t n_sp02;					//SPO2值
static int32_t n_heart_rate;			//心率值
static int8_t ch_spo2_valid;   //用于显示SP02计算是否有效的指示符
static int8_t  ch_hr_valid;    //用于显示心率计算是否有效的指示符
static uint32_t un_min, un_max, un_prev_data;  

static void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer ,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer ,   int32_t *pn_spo2, int8_t *pch_spo2_valid ,  int32_t *pn_heart_rate , int8_t  *pch_hr_valid);
static void maxim_find_peaks( int32_t *pn_locs, int32_t *pn_npks,  int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num );
static void maxim_peaks_above_min_height( int32_t *pn_locs, int32_t *pn_npks,  int32_t *pn_x, int32_t n_size, int32_t n_min_height );
static void maxim_remove_close_peaks( int32_t *pn_locs, int32_t *pn_npks,   int32_t  *pn_x, int32_t n_min_distance );
static void maxim_sort_ascend( int32_t *pn_x, int32_t n_size );
static void maxim_sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);

/******************************************************************************
 * @brief	初始化MAX30102
 * @param	dev	:  MAX30102Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int max30102_init(MAX30102Dev_t *dev)
{
    if (!dev)
		return -1;

	uint16_t i;

    /* 保存私有数据 */
	dev->priv_data = (MAX30102PrivData_t *)malloc(sizeof(MAX30102PrivData_t));
	if (!dev->priv_data)
		return -1;
	
	MAX30102PrivData_t *priv_data = (MAX30102PrivData_t *)dev->priv_data;
	
	priv_data->i2c.info.scl_port = dev->info.scl_port;
	priv_data->i2c.info.scl_pin = dev->info.scl_pin;
	priv_data->i2c.info.sda_port = dev->info.sda_port;
	priv_data->i2c.info.sda_pin = dev->info.sda_pin;

	/* 配置MAX30102 */
	__max30102_config_gpio_clock_enable(dev->info.int_port);
	__max30102_config_io_in_up(dev->info.int_port, dev->info.int_pin);
	
	/* 配置软件I2C */
	i2c_init(&priv_data->i2c);

	dev->init_flag = true;

	/* MAX30102硬件初始化 */
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_MODE_CONFIG, 0x40);		// 复位设备：将模式配置寄存器的RESET位（第6位）置1，触发芯片复位，恢复默认状态
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_INTR_ENABLE_1, 0xC0);	// 配置中断使能
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_INTR_ENABLE_2, 0x00);		
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_FIFO_WR_PTR, 0x00);		// 初始化FIFO写入指针为0		
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_OVF_COUNTER, 0x00);   	// 初始化溢出数据计数器为0
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_FIFO_RD_PTR, 0x00);   	// 初始化FIFO读取指针为0
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_FIFO_CONFIG, 0x0F);   	// 无平均取样,当FIFO完全充满数据时,FIFO地址滚动到零并且FIFO继续填充新数据
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_MODE_CONFIG, 0x03);    	// SpO2模式
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_SPO2_CONFIG, 0x27);   	// ADC量程4096nA，采样率100Hz，18位分辨率
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_LED1_PA, 0x24);			// LED1电流设置为7.4mA
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_LED2_PA, 0x24);			// LED2电流设置为7.4mA
	priv_data->i2c.write_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_PILOT_PA, 0x7F);
	
	/* MAX30102软件初始化 */
	un_min = 0x3FFFF;
	un_max = 0;

	/* 读取前500个样本，并确定信号范围 */
	for (i = 0; i < BUFFER_SIZE; i++)
	{
		while (__max30102_io_read(dev->info.int_port, dev->info.int_pin) == 1);
			
		__max30102_read_fifo(dev, &aun_red_buffer[i], &aun_ir_buffer[i]);
				
		if (un_min > aun_red_buffer[i])
			un_min = aun_red_buffer[i];    // 更新计算最小值
		if (un_max < aun_red_buffer[i])
			un_max = aun_red_buffer[i];    // 更新计算最大值
	}
	un_prev_data = aun_red_buffer[i];

	/* 计算前500个样本后的心率和血氧饱和度 */
	maxim_heart_rate_and_oxygen_saturation(	aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, 
											&n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
	
    /* 函数指针赋值 */
	dev->get_data = __max30102_get_data;
	dev->deinit = __max30102_deinit;
    
	return 0;
}

/******************************************************************************
 * @brief	从MAX30102的FIFO寄存器读取一组样本
 * @param	dev    			:   MAX30102Dev_t 结构体指针
 * @param	red_led_data    :   红色LED读取数据的指针
 * @param	ir_led_data    	:   红外LED读取数据的指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __max30102_read_fifo(MAX30102Dev_t *dev, uint32_t *red_led_data, uint32_t *ir_led_data)
{
	if (!dev || !dev->init_flag)
 		return -1;

	uint8_t uch_temp;
	uint8_t uch_i2c_data[6];
	uint8_t mode;
	int ret;

	MAX30102PrivData_t *priv_data = (MAX30102PrivData_t *)dev->priv_data;

	/* 读取并清除状态寄存器 */
	priv_data->i2c.read_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_INTR_STATUS_1, &uch_temp);
	priv_data->i2c.read_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_INTR_STATUS_2, &uch_temp);

	/* 从FIFO中读取6个字节的数据存入数组 */
	ret = priv_data->i2c.read_regs(&priv_data->i2c, MAX30102_ADDRESS, REG_FIFO_DATA, 6, uch_i2c_data);
	if (ret != 0)
	{
		return ret;
	}

	*ir_led_data = 0UL;
	*red_led_data = 0UL;

	/* 根据工作模式解析数据顺序 */
    priv_data->i2c.read_reg(&priv_data->i2c, MAX30102_ADDRESS, REG_MODE_CONFIG, &mode);
    if (mode == 0x03)
	{
		*red_led_data = ((uint32_t)uch_i2c_data[0] << 16) | (uch_i2c_data[1] << 8) | uch_i2c_data[2];
        *ir_led_data = ((uint32_t)uch_i2c_data[3] << 16) | (uch_i2c_data[4] << 8) | uch_i2c_data[5];
	}
	else
	{
		return -2;
	}
	
	/* 屏蔽无效高位[23:18] */
	*red_led_data &= 0x03FFFF;
	*ir_led_data &= 0x03FFFF;

	return 0;
}

/******************************************************************************
 * @brief	MAX30102获取心率血氧数据
 * @param	dev    			:   MAX30102Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __max30102_get_data(MAX30102Dev_t *dev)
{
    if (!dev || !dev->init_flag)
 		return -1;
    
	int32_t n_brightness;
	float f_temp;
	uint16_t i;
	uint8_t dis_hr = 0, dis_spo2 = 0;

	/* 舍去前100组样本，并将后400组样本移到顶部，将100~500缓存数据移位到0~400 */
	for (i = 100; i < BUFFER_SIZE; i++)
	{
		aun_red_buffer[i - 100] = aun_red_buffer[i];	// 将100-500缓存数据移位到0-400
		aun_ir_buffer[i - 100] = aun_ir_buffer[i];		// 将100-500缓存数据移位到0-400
		
		// update the signal min and max
		if (un_min > aun_red_buffer[i])			// 寻找移位后0-400中的最小值
			un_min = aun_red_buffer[i];
		if (un_max < aun_red_buffer[i])			// 寻找移位后0-400中的最大值
			un_max = aun_red_buffer[i];
	}
	
	/* 在计算心率前取100组样本，取的数据放在400-500缓存数组中 */
	for (i = 400; i < BUFFER_SIZE; i++)
	{
			un_prev_data=aun_red_buffer[i - 1];	// 在计算心率前取100组样本，取的数据放在400-500缓存数组中
			
			while (__max30102_io_read(dev->info.int_port, dev->info.int_pin) == 1);
			
			__max30102_read_fifo(dev, &aun_red_buffer[i], &aun_ir_buffer[i]);

			if (aun_red_buffer[i] > un_prev_data)		// 用新获取的一个数值与上一个数值对比
			{
				f_temp = aun_red_buffer[i]-  un_prev_data;
				f_temp /= (un_max - un_min);
				f_temp *= MAX_BRIGHTNESS;				// 公式（心率曲线）=（新数值-旧数值）/（最大值-最小值）*255
				n_brightness -= (int)f_temp;
				if (n_brightness < 0)
					n_brightness = 0;
			}
			else
			{
				f_temp  =un_prev_data - aun_red_buffer[i];
				f_temp /= (un_max - un_min);
				f_temp *= MAX_BRIGHTNESS;				// 公式（心率曲线）=（旧数值-新数值）/（最大值-最小值）*255
				n_brightness += (int)f_temp;
				if (n_brightness > MAX_BRIGHTNESS)
					n_brightness = MAX_BRIGHTNESS;
			}
	
		if (ch_hr_valid == 1 && n_heart_rate < 140)
		{
			dis_hr = n_heart_rate;
			dis_spo2 = n_sp02;
		}
		else
		{
			dis_hr = 0;
			dis_spo2 = 0;
		}
	}
	maxim_heart_rate_and_oxygen_saturation(	aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, 
											&n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

	if (dis_hr > 0 && dis_hr < 140)
		dev->heart_rate = dis_hr - 20;

	if (dis_spo2 > 0 && dis_hr < 100)
		dev->blood_oxygen = dis_spo2;

	return 0;
}

/******************************************************************************
 * @brief	去初始化MAX30102
 * @param	dev   :  MAX30102Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __max30102_deinit(MAX30102Dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;

	MAX30102PrivData_t *priv_data = (MAX30102PrivData_t *)dev->priv_data;
	
	/* 去初始化软件I2C */
	priv_data->i2c.deinit(&priv_data->i2c);

	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志

    return 0;
}

/*****************************MAX30102算法部分**********************************/

const uint16_t auw_hamm[31]={41, 276, 512, 276, 41}; // Hamm = long16(512* hamming(5)');
const uint8_t uch_spo2_table[184]={	95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
									99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
									100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
									97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
									90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
									80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
									66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
									49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
									28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
									3, 2, 1} ;
static  int32_t an_dx[ BUFFER_SIZE-MA4_SIZE]; // delta
static  int32_t an_x[ BUFFER_SIZE]; //ir
static  int32_t an_y[ BUFFER_SIZE]; //red

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, 
                              int32_t *pn_heart_rate, int8_t  *pch_hr_valid)
/**
* \brief        Calculate the heart rate and SpO2 level
* \par          Details
*               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the ratio for the SPO2 is computed.
*               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
*               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each ratio.
*
* \param[in]    *pun_ir_buffer           - IR sensor data buffer
* \param[in]    n_ir_buffer_length      - IR sensor data buffer length
* \param[in]    *pun_red_buffer          - Red sensor data buffer
* \param[out]    *pn_spo2                - Calculated SpO2 value
* \param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
* \param[out]    *pn_heart_rate          - Calculated heart rate value
* \param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
*
* \retval       None
*/
{
    uint32_t un_ir_mean ,un_only_once ;
    int32_t k ,n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count ,n_middle_idx;
    int32_t n_th1, n_npks,n_c_min;      
    int32_t an_ir_valley_locs[15] ;
    int32_t an_exact_ir_valley_locs[15] ;
    int32_t an_dx_peak_locs[15] ;
    int32_t n_peak_interval_sum;
    
    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc; 
    int32_t n_y_dc_max, n_x_dc_max; 
    int32_t n_y_dc_max_idx, n_x_dc_max_idx; 
    int32_t an_ratio[5],n_ratio_average; 
    int32_t n_nume,  n_denom ;
    // remove DC of ir signal    
    un_ir_mean =0; 
    for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
    un_ir_mean =un_ir_mean/n_ir_buffer_length ;
    for (k=0 ; k<n_ir_buffer_length ; k++ )  an_x[k] =  pun_ir_buffer[k] - un_ir_mean ; 
    
    // 4 pt Moving Average
    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
        n_denom= ( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3]);
        an_x[k]=  n_denom/(int32_t)4; 
    }

    // get difference of smoothed IR signal
    
    for( k=0; k<BUFFER_SIZE-MA4_SIZE-1;  k++)
        an_dx[k]= (an_x[k+1]- an_x[k]);

    // 2-pt Moving Average to an_dx
    for(k=0; k< BUFFER_SIZE-MA4_SIZE-2; k++){
        an_dx[k] =  ( an_dx[k]+an_dx[k+1])/2 ;
    }
    
    // hamming window
    // flip wave form so that we can detect valley with peak detector
    for ( i=0 ; i<BUFFER_SIZE-HAMMING_SIZE-MA4_SIZE-2 ;i++){
        s= 0;
        for( k=i; k<i+ HAMMING_SIZE ;k++){
            s -= an_dx[k] *auw_hamm[k-i] ; 
                     }
        an_dx[i]= s/ (int32_t)1146; // divide by sum of auw_hamm 
    }

 
    n_th1=0; // threshold calculation
    for ( k=0 ; k<BUFFER_SIZE-HAMMING_SIZE ;k++){
        n_th1 += ((an_dx[k]>0)? an_dx[k] : ((int32_t)0-an_dx[k])) ;
    }
    n_th1= n_th1/ ( BUFFER_SIZE-HAMMING_SIZE);
    // peak location is acutally index for sharpest location of raw signal since we flipped the signal         
    maxim_find_peaks( an_dx_peak_locs, &n_npks, an_dx, BUFFER_SIZE-HAMMING_SIZE, n_th1, 8, 5 );//peak_height, peak_distance, max_num_peaks 

    n_peak_interval_sum =0;
    if (n_npks>=2){
        for (k=1; k<n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k]-an_dx_peak_locs[k -1]);
        n_peak_interval_sum=n_peak_interval_sum/(n_npks-1);
        *pn_heart_rate=(int32_t)(6000/n_peak_interval_sum);// beats per minutes
        *pch_hr_valid  = 1;
    }
    else  {
        *pn_heart_rate = -999;
        *pch_hr_valid  = 0;
    }
            
    for ( k=0 ; k<n_npks ;k++)
        an_ir_valley_locs[k]=an_dx_peak_locs[k]+HAMMING_SIZE/2; 


    // raw value : RED(=y) and IR(=X)
    // we need to assess DC and AC value of ir and red PPG. 
    for (k=0 ; k<n_ir_buffer_length ; k++ )  {
        an_x[k] =  pun_ir_buffer[k] ; 
        an_y[k] =  pun_red_buffer[k] ; 
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count =0; 
    for(k=0 ; k<n_npks ;k++){
        un_only_once =1;
        m=an_ir_valley_locs[k];
        n_c_min= 16777216;//2^24;
        if (m+5 <  BUFFER_SIZE-HAMMING_SIZE  && m-5 >0){
            for(i= m-5;i<m+5; i++)
                if (an_x[i]<n_c_min){
                    if (un_only_once >0){
                       un_only_once =0;
                   } 
                   n_c_min= an_x[i] ;
                   an_exact_ir_valley_locs[k]=i;
                }
            if (un_only_once ==0)
                n_exact_ir_valley_locs_count ++ ;
        }
    }
    if (n_exact_ir_valley_locs_count <2 ){
       *pn_spo2 =  -999 ; // do not use SPO2 since signal ratio is out of range
       *pch_spo2_valid  = 0; 
       return;
    }
    // 4 pt MA
    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
        an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int32_t)4;
        an_y[k]=( an_y[k]+an_y[k+1]+ an_y[k+2]+ an_y[k+3])/(int32_t)4;
    }

    //using an_exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration ratio
    //finding AC/DC maximum of raw ir * red between two valley locations
    n_ratio_average =0; 
    n_i_ratio_count =0; 
    
    for(k=0; k< 5; k++) an_ratio[k]=0;
    for (k=0; k< n_exact_ir_valley_locs_count; k++){
        if (an_exact_ir_valley_locs[k] > BUFFER_SIZE ){             
            *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid  = 0; 
            return;
        }
    }
    // find max between two valley locations 
    // and use ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2 

    for (k=0; k< n_exact_ir_valley_locs_count-1; k++){
        n_y_dc_max= -16777216 ; 
        n_x_dc_max= - 16777216; 
        if (an_exact_ir_valley_locs[k+1]-an_exact_ir_valley_locs[k] >10){
            for (i=an_exact_ir_valley_locs[k]; i< an_exact_ir_valley_locs[k+1]; i++){
                if (an_x[i]> n_x_dc_max) {n_x_dc_max =an_x[i];n_x_dc_max_idx =i; }
                if (an_y[i]> n_y_dc_max) {n_y_dc_max =an_y[i];n_y_dc_max_idx=i;}
            }
            n_y_ac= (an_y[an_exact_ir_valley_locs[k+1]] - an_y[an_exact_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_exact_ir_valley_locs[k]); //red
            n_y_ac=  an_y[an_exact_ir_valley_locs[k]] + n_y_ac/ (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k])  ; 
        
        
            n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw 
            n_x_ac= (an_x[an_exact_ir_valley_locs[k+1]] - an_x[an_exact_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_exact_ir_valley_locs[k]); // ir
            n_x_ac=  an_x[an_exact_ir_valley_locs[k]] + n_x_ac/ (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k]); 
            n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw 
            n_nume=( n_y_ac *n_x_dc_max)>>7 ; //prepare X100 to preserve floating value
            n_denom= ( n_x_ac *n_y_dc_max)>>7;
            if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
            {   
                an_ratio[n_i_ratio_count]= (n_nume*20)/n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;  ///*************************n_nume原来是*100************************//
                n_i_ratio_count++;
            }
        }
    }

    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx= n_i_ratio_count/2;

    if (n_middle_idx >1)
        n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx ];

    if( n_ratio_average>2 && n_ratio_average <184){
        n_spo2_calc= uch_spo2_table[n_ratio_average] ;
        *pn_spo2 = n_spo2_calc ;
        *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
    }
    else{
        *pn_spo2 =  -999 ; // do not use SPO2 since signal ratio is out of range
        *pch_spo2_valid  = 0; 
    }
}


void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
{
    maxim_peaks_above_min_height( pn_locs, pn_npks, pn_x, n_size, n_min_height );
    maxim_remove_close_peaks( pn_locs, pn_npks, pn_x, n_min_distance );
    *pn_npks = min( *pn_npks, n_max_num );
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t  *pn_x, int32_t n_size, int32_t n_min_height)
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
    int32_t i = 1, n_width;
    *pn_npks = 0;
    
    while (i < n_size-1){
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){            // find left edge of potential peaks
            n_width = 1;
            while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])    // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i+n_width] && (*pn_npks) < 15 ){                            // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;        
                // for flat peaks, peak location is left edge
                i += n_width+1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}


void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{
    
    int32_t i, j, n_old_npks, n_dist;
    
    /* Order peaks from large to small */
    maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

    for ( i = -1; i < *pn_npks; i++ ){
        n_old_npks = *pn_npks;
        *pn_npks = i+1;
        for ( j = i+1; j < n_old_npks; j++ ){
            n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
            if ( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices longo ascending order
    maxim_sort_ascend( pn_locs, *pn_npks );
}

void maxim_sort_ascend(int32_t *pn_x,int32_t n_size) 
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/ 
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
            pn_indx[j] = pn_indx[j-1];
        pn_indx[j] = n_temp;
    }
}
