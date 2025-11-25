#include "drv_key.h"
#include <stddef.h>
#include <string.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void key_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_KEY_PLATFORM_STM32F1
#define RCC_CMD(port) RCC_APB2PeriphClockCmd(RCC_APB2Periph_##port, ENABLE)
#elif DRV_KEY_PLATFORM_STM32F4
#define RCC_CMD(port) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_##port, ENABLE)
#elif DRV_KEY_PLATFORM_GD32F1
#define RCC_CMD(port) rcu_periph_clock_enable(RCU_##port)
#endif
	switch ((uint32_t)port) {
    case (uint32_t)GPIOA: RCC_CMD(GPIOA); break;
    case (uint32_t)GPIOB: RCC_CMD(GPIOB); break;
	case (uint32_t)GPIOC: RCC_CMD(GPIOC); break;
	case (uint32_t)GPIOD: RCC_CMD(GPIOD); break;
	case (uint32_t)GPIOE: RCC_CMD(GPIOE); break;
	case (uint32_t)GPIOF: RCC_CMD(GPIOF); break;
	case (uint32_t)GPIOG: RCC_CMD(GPIOG); break;
	}
}

/**
 * @brief	初始化 GPIO
 * @param[in] cfg key_cfg_t 结构体指针
 */
static void key_hw_gpio_init(const key_cfg_t *cfg)
{
#if DRV_KEY_PLATFORM_STM32F1
    GPIO_InitTypeDef GPIO_InitStructure;
	if (cfg->press_level)
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	else
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_KEY_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	if (cfg->press_level)
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	else
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->pin;
	GPIO_Init(cfg->port, &GPIO_InitStructure);

#elif DRV_KEY_PLATFORM_GD32F1
	if (cfg->press_level)
		gpio_init(cfg->port, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, cfg->pin);
	else
		gpio_init(cfg->port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, cfg->pin);
#endif
}

/**
 * @brief	GPIO 读输入引脚电平
 * @param[in] port 端口
 * @param[in] pin  引脚
 * @return	电平
 */
static inline uint8_t key_hw_gpio_read_in_bit(gpio_port_t port, gpio_pin_t pin)
{
#if DRV_KEY_PLATFORM_STM32F1 || DRV_KEY_PLATFORM_STM32F4
	return GPIO_ReadInputDataBit(port, pin);
#elif DRV_KEY_PLATFORM_GD32F1
	return (uint8_t)gpio_input_bit_get(port, pin);
#endif
}

/**
 * @brief   初始化按键硬件
 * @param[in] cfg key_cfg_t 结构体指针
 */
static void key_hw_init(const key_cfg_t *cfg)
{	
	key_hw_gpio_clock_enable(cfg->port);
	key_hw_gpio_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

/* 按键状态标志位 */
typedef enum {
    KEY_FLAG_NONE   = 0x00,
    KEY_FLAG_DOWN   = 0x01,
    KEY_FLAG_UP     = 0x02,
    KEY_FLAG_CLICK  = 0x04,
	KEY_FLAG_DOUBLE = 0x08,
	KEY_FLAG_LONG   = 0x10,
	KEY_FLAG_REPEAT = 0x20
} key_flag_t;

/* 按键按下与释放状态 */
typedef enum {
    KEY_PRESS_STATE_RELEASED = 0,
    KEY_PRESS_STATE_PRESSED  = 1
} key_press_state_t;

/* 按键状态机状态 */
typedef enum {
    KEY_FSM_STATE_IDLE = 0,	// 空闲状态（未按下）
    KEY_FSM_STATE_PRESSED,	// 已按下（等待长按或释放）
    KEY_FSM_STATE_RELEASED,	// 已释放（等待双击或单击判定）
    KEY_FSM_STATE_DOUBLE, 	// 已触发双击（等待释放）
    KEY_FSM_STATE_LONG    	// 已触发长按（等待重复或释放）
} key_fsm_state_t;

/* 按键事件结构体 */
typedef struct {
    key_event_callback_t callback;
    void *param;
} key_event_t;

/* 环形缓冲区 */
#define NEXT_POS(x) ((x + 1) % KEY_BUF_LEN)

typedef struct {
	key_flag_t flag[KEY_BUF_LEN];
	int read;
	int write;
} key_buf_t;

/* 私有数据结构体 */
typedef struct {
	key_dev_t 		 *dev;
	key_buf_t 		  buf;
	key_event_t 	  events[KEY_EVENT_MAX];
	uint16_t 		  debounce_cnt;		// 消抖计数器
    key_press_state_t pre_press_state;	// 上次按压状态
    key_press_state_t cur_press_state;	// 本次按压状态
	key_fsm_state_t	  fsm_state;		// 状态机状态
	uint16_t 		  fsm_timer;		// 状态机计时器
	bool    		  in_use;
} key_priv_t;

static key_priv_t g_key_priv[MAX_KEY_NUM];

static key_priv_t *key_priv_alloc(void);
static void key_priv_free(key_priv_t *priv);
static bool key_buf_is_empty(key_buf_t *buf);
static bool key_buf_is_full(key_buf_t *buf);
static void key_buf_write(key_buf_t *buf, key_flag_t flag);
static key_flag_t key_buf_read(key_buf_t *buf);
static key_press_state_t key_get_status(key_dev_t *dev);
static int key_register_callback_impl(key_dev_t *dev, 
									  key_event_type_t type, 
                                      key_event_callback_t callback, 
									  void *param);
static int key_process_event_impl(key_dev_t *dev);
static int key_clear_event_impl(key_dev_t *dev);
static int key_deinit_impl(key_dev_t *dev);

/* 操作接口表 */
static const key_ops_t key_ops = {
	.register_callback = key_register_callback_impl,
	.process_event 	   = key_process_event_impl,
	.clear_event       = key_clear_event_impl,
	.deinit 	       = key_deinit_impl
};

/**
 * @brief   初始化按键设备驱动
 * @param[out] dev key_dev_t 结构体指针
 * @param[in]  cfg key_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_key_init(key_dev_t *dev, const key_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	key_priv_t *priv = key_priv_alloc();
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->buf.read = 0;
	priv->buf.write = 0;

	dev->priv = priv;
	dev->cfg  = *cfg;
	dev->ops  = &key_ops;
	
	key_hw_init(cfg);
	return 0;
}

/**
 * @brief   按键扫描函数，必须 1ms 调用一次
 * @details 每次调用会遍历所有注册的按键，执行以下操作：
 * 			1. 消抖处理；
 * 			2. 记录当前按键状态；
 * 			3. 进行有限状态机（FSM）状态迁移；
 * 			4. 生成对应的按键事件（DOWN / UP / CLICK / DOUBLE / LONG / REPEAT）。
 * @param[in] param 未使用参数（传入 NULL）
 */
void drv_key_tick(void *param)
{
    uint8_t i;
	key_flag_t flag;
	key_priv_t *priv;	

	for (i = 0; i < MAX_KEY_NUM; i++) {
		priv = &g_key_priv[i];
		if (!priv->in_use)
			continue;	// 跳过未启用的按键

		/* 状态机计时器递减 */
		if (priv->fsm_timer > 0)
			priv->fsm_timer--;

		/* 消抖计数，未到达阈值则继续累积 */
		if (++priv->debounce_cnt < priv->dev->cfg.debounce_ms)
			continue;
		priv->debounce_cnt = 0;

		/* 记录前后两次扫描的按键电平状态 */
        priv->pre_press_state = priv->cur_press_state;
		priv->cur_press_state = key_get_status(priv->dev);

        flag = KEY_FLAG_NONE;

		/* 边沿检测：设置按下与释放事件 */
        if (priv->pre_press_state == KEY_PRESS_STATE_RELEASED && 
			priv->cur_press_state == KEY_PRESS_STATE_PRESSED)
            flag |= KEY_FLAG_DOWN;
        else if (priv->pre_press_state == KEY_PRESS_STATE_PRESSED && 
				 priv->cur_press_state == KEY_PRESS_STATE_RELEASED)
            flag |= KEY_FLAG_UP;

		/* 按键状态机：管理单击、双击、长按等逻辑 */
		switch (priv->fsm_state) {
		case KEY_FSM_STATE_IDLE:
			/* 空闲状态检测到按下进入 PRESSED */
			if (priv->cur_press_state == KEY_PRESS_STATE_PRESSED) {
				priv->fsm_timer = priv->dev->cfg.long_timeout_ms;
				priv->fsm_state = KEY_FSM_STATE_PRESSED;
			}
			break;
		case KEY_FSM_STATE_PRESSED:
			if (priv->cur_press_state == KEY_PRESS_STATE_RELEASED) {
				/* 按下后释放 → 单击或双击 */
				if (priv->dev->cfg.enable_double) {
					priv->fsm_timer = priv->dev->cfg.double_timeout_ms;
					priv->fsm_state = KEY_FSM_STATE_RELEASED;
				} else {
					flag |= KEY_FLAG_CLICK;
					priv->fsm_state = KEY_FSM_STATE_IDLE;
				}
			} else if (priv->fsm_timer == 0) {
				/* 直到超时都未释放 → 长按 */
				priv->fsm_timer = priv->dev->cfg.repeat_timeout_ms;
				if (priv->dev->cfg.enable_long)
					flag |= KEY_FLAG_LONG;
				priv->fsm_state = KEY_FSM_STATE_LONG;
			}
			break;
		case KEY_FSM_STATE_RELEASED:
			if (priv->cur_press_state == KEY_PRESS_STATE_PRESSED) {
				/* 释放后再次按下 → 双击 */
				if (priv->dev->cfg.enable_double)
					flag |= KEY_FLAG_DOUBLE;
				priv->fsm_timer = 0;
				priv->fsm_state = KEY_FSM_STATE_DOUBLE;
			} else if (priv->fsm_timer == 0) {
				/* 超时未二次按下 → 单击 */
				flag |= KEY_FLAG_CLICK;
				priv->fsm_state = KEY_FSM_STATE_IDLE;
			}
			break;
		case KEY_FSM_STATE_DOUBLE:
			/* 双击完成后等待释放 */
			if (priv->cur_press_state == KEY_PRESS_STATE_RELEASED)
				priv->fsm_state = KEY_FSM_STATE_IDLE;
			break;
		case KEY_FSM_STATE_LONG:
			if (priv->cur_press_state == KEY_PRESS_STATE_RELEASED) {
				/* 长按结束 */
				priv->fsm_state = KEY_FSM_STATE_IDLE;
			} else if (priv->fsm_timer == 0) {
				/* 一直长按 → 重复按 */
				priv->fsm_timer = priv->dev->cfg.repeat_timeout_ms;
				if (priv->dev->cfg.enable_repeat)
					flag |= KEY_FLAG_REPEAT;
				priv->fsm_state = KEY_FSM_STATE_LONG;
			}
			break;
		default:
			break;
		}

        /* 状态变化时才写入事件缓冲区 */
        if (flag != KEY_FLAG_NONE)
            key_buf_write(&priv->buf, flag);
    }
}

/**
 * @brief   从私有数据数组中分配一个空闲槽位
 * @return	成功返回槽位指针，失败返回 NULL
 */
static key_priv_t *key_priv_alloc(void)
{
	for (uint8_t i = 0; i < MAX_KEY_NUM; i++) {
		if (!g_key_priv[i].in_use) {
			g_key_priv[i].in_use = true;
            g_key_priv[i].debounce_cnt = 0;
            g_key_priv[i].pre_press_state = KEY_PRESS_STATE_RELEASED;
            g_key_priv[i].cur_press_state = KEY_PRESS_STATE_RELEASED;
            g_key_priv[i].fsm_state = KEY_FSM_STATE_IDLE;
            g_key_priv[i].fsm_timer = 0;
            g_key_priv[i].buf.read = 0;
            g_key_priv[i].buf.write = 0;
            memset(g_key_priv[i].events, 0, sizeof(g_key_priv[i].events));
			return &g_key_priv[i];
		}
	}
	return NULL;
}

/**
 * @brief   释放私有数据槽位
 * @param[in,out] priv 待释放的槽位 key_priv_t 结构体指针
 */
static void key_priv_free(key_priv_t *priv)
{
	if (priv)
		priv->in_use = false;
}

/**
 * @brief   判断环形缓冲区空
 * @param[in] buf key_buf_t 结构体指针
 * @return	true 表示空, false 表示非空
 */
static bool key_buf_is_empty(key_buf_t *buf)
{
	return (buf->read == buf->write);
}

/**
 * @brief   判断环形缓冲区满
 * @param[in] buf key_buf_t 结构体指针
 * @return	true 表示满, false 表示非满
 */
static bool key_buf_is_full(key_buf_t *buf)
{
	return (buf->read == NEXT_POS(buf->write));
}

/**
 * @brief   环形缓冲区写数据
 * @param[in] buf  key_buf_t 结构体指针
 * @param[in] flag 按键状态标志位
 */
static void key_buf_write(key_buf_t *buf, key_flag_t flag)
{
	if (!key_buf_is_full(buf)) {
		buf->flag[buf->write] = flag;
		buf->write = NEXT_POS(buf->write);
	}
}

/**
 * @brief   环形缓冲区读数据
 * @param[in] buf key_buf_t 结构体指针
 * @return	读取到的按键状态标志位
 */
static key_flag_t key_buf_read(key_buf_t *buf)
{
	key_flag_t flag = KEY_FLAG_NONE;

	if (!key_buf_is_empty(buf)) {
		flag = buf->flag[buf->read];
		buf->read = NEXT_POS(buf->read);
	}
	return flag;
}

/**
 * @brief   获取按键状态
 * @param[in] dev 	 key_dev_t 结构体指针
 * @return	按键状态
 */
static key_press_state_t key_get_status(key_dev_t *dev)
{
	uint8_t status = key_hw_gpio_read_in_bit(dev->cfg.port, dev->cfg.pin);

	if (status == dev->cfg.press_level)
		return KEY_PRESS_STATE_PRESSED;
	else
		return KEY_PRESS_STATE_RELEASED;
}

/**
 * @brief   按键注册事件回调函数
 * @param[in] dev      key_dev_t 结构体指针
 * @param[in] type     按键事件类型
 * @param[in] callback 事件回调函数
 * @param[in] param    事件回调函数参数
 * @return	0 表示成功，其他值表示失败
 */
static int key_register_callback_impl(key_dev_t *dev, 
									  key_event_type_t type, 
                                      key_event_callback_t callback, 
									  void *param)
{
    if (!dev || type >= KEY_EVENT_MAX)
        return -EINVAL;

	key_priv_t *priv = (key_priv_t *)dev->priv;
	
	priv->events[type].callback = callback;
    priv->events[type].param 	= param;
    return 0;
}

/**
 * @brief   按键处理事件
 * @param[in] dev key_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int key_process_event_impl(key_dev_t *dev)
{
	if (!dev)
		return -EINVAL;
	
	uint8_t i;
	key_flag_t flag;
	key_priv_t *priv = (key_priv_t *)dev->priv;
	key_event_type_t type;

	/* 定义事件类型与标志位的映射关系表 */
	static const struct {
		key_event_type_t event_type;
		key_flag_t flag;
	} event_flag_map[] = {
		{ KEY_EVENT_DOWN,   KEY_FLAG_DOWN   },
		{ KEY_EVENT_UP,     KEY_FLAG_UP     },
		{ KEY_EVENT_CLICK,  KEY_FLAG_CLICK  },
		{ KEY_EVENT_DOUBLE, KEY_FLAG_DOUBLE },
		{ KEY_EVENT_LONG,   KEY_FLAG_LONG   },
		{ KEY_EVENT_REPEAT, KEY_FLAG_REPEAT },
	};

	/* 从环形缓冲区中读取出标志位，针对不同事件类型调用回调函数 */
    while ((flag = key_buf_read(&priv->buf)) != 0) {
		for (i = 0; i < sizeof(event_flag_map) / sizeof(event_flag_map[0]); i++) {
            type = event_flag_map[i].event_type;
            
            if ((flag & event_flag_map[i].flag) && priv->events[type].callback) {
                priv->events[type].callback(priv->events[type].param);
            }
        }
    }
	
	return 0;
}

/**
 * @brief   按键清空事件，即清空缓冲区，丢弃所有未处理的事件
 * @param[in] dev key_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int key_clear_event_impl(key_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	key_priv_t *priv = (key_priv_t *)dev->priv;
    priv->buf.read = priv->buf.write;
    return 0;
}

/**
 * @brief   去初始化按键
 * @param[in] dev key_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int key_deinit_impl(key_dev_t *dev)
{
	if (!dev)
		return -EINVAL;
	
	key_priv_t *priv = (key_priv_t *)dev->priv;
	key_priv_free(priv);
	dev->priv = NULL;
	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
