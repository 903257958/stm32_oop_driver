#include "bsp.h"

int main(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    bsp_init();
	
	while (1) {
		sr04.get_distance(&sr04);
        debug.printf("Distance: %.2f cm\r\n", sr04.distance_cm);
        delay_ms(100);
	}
}
