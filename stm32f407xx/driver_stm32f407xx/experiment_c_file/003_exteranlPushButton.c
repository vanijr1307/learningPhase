/*
 * 003_exteranlPushButton.c
 *
 *  Created on: Oct 17, 2025
 *      Author: vani.jr
 */



#include  "driver_stm32f407xx.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED HIGH
void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
int main(void){
	GPIO_Handle_t GpioLed,GpioBtn;

	GpioLed.pGPIOx =GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

//	while(1){
//		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
//		delay();
//	}


	GpioBtn.pGPIOx =GPIOB;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


		GPIO_PeriClockControl(GPIOB,ENABLE);
		GPIO_Init(&GpioBtn);

		while(1){
			if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12)==BTN_PRESSED){
				delay();
				GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
			}
		}
	return 0;
}

