#include "bsp.h"

int main(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    bsp_init();

	while (1);
}
