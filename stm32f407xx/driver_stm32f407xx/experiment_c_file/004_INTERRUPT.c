/*
 * 002_ledButton.c
 *
 *  Created on: Oct 17, 2025
 *      Author: vani.jr
 */



#include  "driver_stm32f407xx.h"
#include<string.h>

#define HIGH 1
#define BTN_PRESSED HIGH
void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
int main(void){
	GPIO_Handle_t GpioLed,GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn)); // to make very members in the set intialized to 0

	GpioLed.pGPIOx =GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;//if open drain use inetrnal or external push or pull otherwise floating state
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

//	while(1){
//		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
//		delay();
//	}


	GpioBtn.pGPIOx =GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


		GPIO_PeriClockControl(GPIOD,ENABLE);
		GPIO_Init(&GpioBtn);

		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, GPIO_PIN_RESET);

		GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15 );
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


		while(1);

//		while(1){
//			if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)==BTN_PRESSED){
//				delay();
//				GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
//			}
//		}
	return 0;
}

void EXTI9_5_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);



}

