#include "bsp.h"

int main(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
	bsp_init();
	
    while (1) {
        led.toggle(&led);
        delay_ms(500);
    }
}
