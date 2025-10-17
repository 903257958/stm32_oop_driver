#include "bsp.h"

/* 测试：TIMER1延时500ms打印，TIMER2中断500ms打印 */
int main(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    bsp_init();
	
	while (1) {
		timer1.delay_ms(&timer1, 500);
        debug.printf("timer1 500ms delay!\r\n");
	}
}
