#include "drv_cst816t.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void cst816t_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_CST816T_PLATFORM_STM32F1
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); break;
	case (uint32_t)GPIOB: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); break;
	case (uint32_t)GPIOC: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); break;
	case (uint32_t)GPIOD: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); break;
	case (uint32_t)GPIOE: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); break;
	case (uint32_t)GPIOF: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); break;
	case (uint32_t)GPIOG: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_CST816T_PLATFORM_STM32F4
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); break;
	case (uint32_t)GPIOB: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); break;
	case (uint32_t)GPIOC: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); break;
	case (uint32_t)GPIOD: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); break;
	case (uint32_t)GPIOE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); break;
	case (uint32_t)GPIOF: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); break;
	case (uint32_t)GPIOG: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_CST816T_PLATFORM_GD32F1
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
	case (uint32_t)GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
	case (uint32_t)GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
	case (uint32_t)GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
	case (uint32_t)GPIOE: rcu_periph_clock_enable(RCU_GPIOE); break;
	case (uint32_t)GPIOF: rcu_periph_clock_enable(RCU_GPIOF); break;
	case (uint32_t)GPIOG: rcu_periph_clock_enable(RCU_GPIOG); break;
	default: break;
    }
#endif
}

/**
 * @brief	初始化 GPIO
 * @param[in] cfg cst816t_cfg_t 结构体指针
 */
static void cst816t_hw_gpio_init(const cst816t_cfg_t *cfg)
{
#if DRV_CST816T_PLATFORM_STM32F1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->rst_pin;
	GPIO_Init(cfg->rst_port, &GPIO_InitStructure);
#elif DRV_CST816T_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->rst_pin;
	GPIO_Init(cfg->rst_port, &GPIO_InitStructure);
#elif DRV_CST816T_PLATFORM_GD32F1
	gpio_init(cfg->rst_port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, cfg->rst_pin);
#endif
}

/**
 * @brief	写 GPIO 引脚电平
 * @param[in] port  端口
 * @param[in] pin   引脚
 * @param[in] level 电平
 */
static inline void cst816t_hw_gpio_write_bit(gpio_port_t port, gpio_pin_t pin, uint8_t level)
{
#if DRV_CST816T_PLATFORM_STM32F1 || DRV_CST816T_PLATFORM_STM32F4
	GPIO_WriteBit(port, pin, (BitAction)level);
#elif DRV_CST816T_PLATFORM_GD32F1
	gpio_bit_write(port, pin, (bit_status)(level));
#endif
}

/**
 * @brief   CST816T 复位
 * @param[in] cfg cst816t_cfg_t 结构体指针
 */
static void cst816t_reset(const cst816t_cfg_t *cfg)
{
    cst816t_hw_gpio_write_bit(cfg->rst_port, cfg->rst_pin, GPIO_LEVEL_LOW);
    cfg->delay_ms(10);
    cst816t_hw_gpio_write_bit(cfg->rst_port, cfg->rst_pin, GPIO_LEVEL_HIGH);
    cfg->delay_ms(50);
}

/**
 * @brief   初始化 CST816T 硬件
 * @param[in] cfg cst816t_cfg_t 结构体指针
 */
static void cst816t_hw_init(const cst816t_cfg_t *cfg)
{
	cst816t_hw_gpio_clock_enable(cfg->rst_port);
	
	cst816t_hw_gpio_init(cfg);
    cst816t_hw_gpio_write_bit(cfg->rst_port, cfg->rst_pin, GPIO_LEVEL_HIGH);

    cst816t_reset(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int cst816t_get_id_impl(cst816t_dev_t *dev, uint8_t *id);
static int cst816t_get_firmware_ver_impl(cst816t_dev_t *dev, uint8_t *fw_ver);
static int cst816t_get_finger_num_impl(cst816t_dev_t *dev, uint8_t *finger_num);
static int cst816t_get_data_impl(cst816t_dev_t *dev, cst816t_data_t *data);
static int cst816t_deinit_impl(cst816t_dev_t *dev);

/* 操作接口表 */
static const cst816t_ops_t cst816t_ops = {
	.get_id           = cst816t_get_id_impl,
    .get_firmware_ver = cst816t_get_firmware_ver_impl,
    .get_finger_num   = cst816t_get_finger_num_impl,
    .get_data         = cst816t_get_data_impl,
	.deinit           = cst816t_deinit_impl,
};

/**
 * @brief   初始化 CST816T 驱动
 * @param[out] dev cst816t_dev_t 结构体指针
 * @param[in]  cfg cst816t_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_cst816t_init(cst816t_dev_t *dev, const cst816t_cfg_t *cfg)
{
    if (!dev || !cfg)
        return -EINVAL;

    dev->cfg = *cfg;
	dev->ops = &cst816t_ops;
	
    cst816t_hw_init(cfg);
	return 0;
}

/**
 * @brief   CST816T 读取 ID
 * @param[in]  dev cst816t_dev_t 结构体指针
 * @param[out] id  ID
 * @return	0 表示成功，其他值表示失败
 */
static int cst816t_get_id_impl(cst816t_dev_t *dev, uint8_t *id)
{
	if (!dev)
		return -EINVAL;

    return dev->cfg.i2c_ops->read_reg(CST816T_ADDRESS, CHIP_ID, id);
}

/**
 * @brief   CST816T 读取固件版本号
 * @param[in]  dev    cst816t_dev_t 结构体指针
 * @param[out] fw_ver 固件版本号
 * @return	0 表示成功，其他值表示失败
 */
static int cst816t_get_firmware_ver_impl(cst816t_dev_t *dev, uint8_t *fw_ver)
{
	if (!dev)
		return -EINVAL;

    return dev->cfg.i2c_ops->read_reg(CST816T_ADDRESS, FW_VER, fw_ver);
}

/**
 * @brief   CST816T 读取触摸手指个数
 * @param[in]  dev        cst816t_dev_t 结构体指针
 * @param[out] finger_num 触摸手指个数
 * @return	0 表示成功，其他值表示失败
 */
static int cst816t_get_finger_num_impl(cst816t_dev_t *dev, uint8_t *finger_num)
{
	if (!dev)
		return -EINVAL;
	
    return dev->cfg.i2c_ops->read_reg(CST816T_ADDRESS, FINGER_NUM, finger_num);
}

/**
 * @brief   CST816T 读当前触摸信息
 * @param[in]  dev  cst816t_dev_t 结构体指针
 * @param[out] data 信息
 * @return	0 表示成功，其他值表示失败
 */
static int cst816t_get_data_impl(cst816t_dev_t *dev, cst816t_data_t *data)
{
	if (!dev)
		return -EINVAL;
	
	uint8_t data_raw[6];
	uint8_t geature_raw;
	uint16_t x, y;
    uint8_t retry_cnt = 0;

    while (retry_cnt < 5) {
        /* 读取寄存器 */
        dev->cfg.i2c_ops->read_regs(CST816T_ADDRESS, GESTURE_ID, 6, data_raw);

        geature_raw = data_raw[0];
        x = ((data_raw[2] & 0x0F) << 8) | data_raw[3];
        y = ((data_raw[4] & 0x0F) << 8) | data_raw[5];

        if (x > dev->cfg.screen_x_max || y > dev->cfg.screen_y_max) {
            cst816t_reset(&dev->cfg);
            retry_cnt++;
            continue;
        }

        /* 解析坐标与动作 */
        if (dev->cfg.is_vertical && dev->cfg.is_forward) {
            data->x = x;
            data->y = y - CST816T_Y_OFFSET;
            switch(geature_raw) {
            case 1: data->gesture = GESTURE_DOWN;	break;
            case 2: data->gesture = GESTURE_UP;		break;
            case 3: data->gesture = GESTURE_LEFT;	break;
            case 4: data->gesture = GESTURE_RIGHT;	break;
            default: break;
            }
            return 0;
        } else if (dev->cfg.is_vertical && !dev->cfg.is_forward) {
            data->x = dev->cfg.screen_x_max - x;
            data->y = dev->cfg.screen_y_max - y;
            switch(geature_raw) {
            case 1: data->gesture = GESTURE_UP;	    break;
            case 2: data->gesture = GESTURE_DOWN;   break;
            case 3: data->gesture = GESTURE_RIGHT;	break;
            case 4: data->gesture = GESTURE_LEFT;	break;
            default: break;
            }
            return 0;
        } else if (!dev->cfg.is_vertical && dev->cfg.is_forward) {
            data->x = y;
            data->y = dev->cfg.screen_x_max - x;
            switch(geature_raw) {
            case 1: data->gesture = GESTURE_RIGHT;  break;
            case 2: data->gesture = GESTURE_LEFT;   break;
            case 3: data->gesture = GESTURE_DOWN;	break;
            case 4: data->gesture = GESTURE_UP;	    break;
            default: break;
            }
            return 0;
        } else if (!dev->cfg.is_vertical && !dev->cfg.is_forward) {
            data->x = dev->cfg.screen_y_max - y;
            data->y = x;
            switch(geature_raw) {
            case 1: data->gesture = GESTURE_LEFT;   break;
            case 2: data->gesture = GESTURE_RIGHT;  break;
            case 3: data->gesture = GESTURE_UP;	    break;
            case 4: data->gesture = GESTURE_DOWN;   break;
            default: break;
            }
            return 0;
        }
    }

	return -3;
}

/**
 * @brief   去初始化 CST816T
 * @param[in] dev cst816t_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int cst816t_deinit_impl(cst816t_dev_t *dev)
{    
    if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
