/*
 * 011_I2C_MASTER_RECEIVE_SLAVETX.c
 *
 *  Created on: Nov 2, 2025
 *      Author: vani.jr
 */


#include<stdio.h>
#include<string.h>
#include "driver_stm32f407xx.h"
extern void initialise_monitor_handles(void);
uint8_t rxComplt = RESET;
I2C_Handle_t I2C1Handle;
#define MY_ADDR   0x61
#define SLAVE_ADDR   0x62
uint8_t rcvBuff[50];

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}
//
//PB6 -SCL
//PB9 -SDA


void I2C1_GPIOInits(void)
{
	GPIO_Handle_t  I2CPins;
    I2CPins.pGPIOx=GPIOB;

     I2CPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
     I2CPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
     I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
     I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode=4;
     I2CPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

//SCL
     I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
     GPIO_Init(&I2CPins);

     //SDA
          I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
          GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
I2C1Handle.pI2Cx=I2C1;
I2C1Handle.I2C_Config.I2C_ACKControl=I2C_ACK_ENABLE;
I2C1Handle.I2C_Config.I2C_DeviceAddress=MY_ADDR;
I2C1Handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
I2C1Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;



	I2C_Init(&I2C1Handle);


}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}



int main(void){

	initialise_monitor_handles();

	uint8_t  commandcode;
	uint8_t  len;
	GPIO_ButtonInit();
	I2C1_GPIOInits();
	I2C1_Inits();
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);
	while(1){

		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		commandcode=0x51;
	while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

	while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

      rxComplt= RESET;
		 commandcode=0x52;
		 while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR)!= I2C_READY);

		 while(I2C_MasterReceiveDataIT(&I2C1Handle, rcvBuff, len, SLAVE_ADDR,I2C_DISABLE_SR)!= I2C_READY);
       while(rxComplt!=SET){}
		 rcvBuff[len-1]='\0';

	     rxComplt= RESET;

	     printf("%d\n",len);
	     //for(uint8_t i=0;i<50;i++)
	     delay();
     printf("data:%s\n",rcvBuff);


	}
//    printf("data:%s\n",rcvBuff);

}

void I2C1_EV_IRQHandler(void){
    I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C1_ER_IRQHandler(void){
    I2C_ER_IRQHandling(&I2C1Handle);

}

void I2C_ApplicationEventCallback(I2C_Handle_t *I2CHandle,uint8_t AppEvt){

	if(AppEvt ==I2C_EV_TX_CMPLT){
		printf("Tx is completed\n");
	}else if(AppEvt ==I2C_EV_RX_CMPLT){
		printf("Rx is completed\n");
		rxComplt =SET;
	}else if(AppEvt ==I2C_ERROR_AF){
		printf("error: ACK failure\n");
		I2C_CloseSendData(I2CHandle);
		I2C_GenerateStopCondition(I2C1);

		while(1);
	}
}



