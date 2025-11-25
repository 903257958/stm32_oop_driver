#include "drv_delay.h"
#include "drv_pwm.h"
#include "drv_servo.h"

static pwm_dev_t pwm;
static const pwm_cfg_t pwm_cfg = {
	.timer_periph = TIM4,
	.oc_channel   = 4,
	.port		  = GPIOB,
	.pin		  = GPIO_Pin_9
};

static int pwm_set_psc(uint16_t psc)
{
	return pwm.ops->set_psc(&pwm, psc);
}
static int pwm_set_arr(uint16_t arr)
{
	return pwm.ops->set_arr(&pwm, arr);
}
static int pwm_set_compare(uint16_t compare)
{
	return pwm.ops->set_compare(&pwm, compare);
}
static servo_pwm_ops_t servo_pwm_ops = {
	.set_psc     = pwm_set_psc,
	.set_arr     = pwm_set_arr,
	.set_compare = pwm_set_compare
};

static servo_dev_t servo;
static const servo_cfg_t servo_cfg = {
    .pwm_ops = &servo_pwm_ops
};

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_pwm_init(&pwm, &pwm_cfg);
    drv_servo_init(&servo, &servo_cfg);

    for (uint8_t i = 0; i <= 180; i += 30) {
        servo.ops->set_angle(&servo, i);
        delay_ms(200);
    }
    for (int16_t i = 180; i >= 0; i -= 30) {
        servo.ops->set_angle(&servo, i);
        delay_ms(200);
    }

	while (1) {
        delay_ms(1000);
	}
}
