/*
 * 002_ledButton.c
 *
 *  Created on: Oct 17, 2025
 *      Author: vani.jr
 */



#include  "driver_stm32f407xx.h"

#define HIGH 1
#define BTN_PRESSED HIGH
void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
int main(void){
	GPIO_Handle_t GpioLed,GpioBtn;

	GpioLed.pGPIOx =GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

//	while(1){
//		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
//		delay();
//	}


	GpioBtn.pGPIOx =GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


		GPIO_PeriClockControl(GPIOA,ENABLE);
		GPIO_Init(&GpioBtn);

		while(1){
			if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)==BTN_PRESSED){
				delay();
				GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
			}
		}
	return 0;
}

