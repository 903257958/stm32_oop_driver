#include "drv_delay.h"
#include "drv_led.h"
#include "drv_timer.h"

static led_dev_t led;
static const led_cfg_t led_cfg = {
    .port         = GPIOC, 
    .pin          = GPIO_Pin_13,
    .active_level = GPIO_LEVEL_LOW,
    .init_state   = LED_STATE_OFF
};

static timer_dev_t timer_delay;
static const timer_cfg_t timer_delay_cfg = {
    .timer_periph = TIM2,
    .psc          = 72 - 1,
    .arr          = 0xFFFF,
    .use_irq      = false
};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    drv_led_init(&led, &led_cfg);
    drv_timer_init(&timer_delay, &timer_delay_cfg);

	while (1) {
		led.ops->toggle(&led);
#if 0
        timer_delay.ops->delay_ms(&timer_delay, 500);
#else
		delay_ms(500);
#endif
	}
}
