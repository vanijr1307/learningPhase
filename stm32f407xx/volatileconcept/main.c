#include<stdint.h>

#define SRAM_Address1   0x20000004U
int main(void){

	uint32_t  value =2;
	uint32_t  volatile *p=(uint32_t*)SRAM_Address1;

	while(1)
	{
		value = *p;
		if(value) break;
	}
	while(1);
	return 0;
}
