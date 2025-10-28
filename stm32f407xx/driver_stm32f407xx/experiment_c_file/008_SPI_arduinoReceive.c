/*
 * 007_SPI_arduinoSlave.c
 *
 *  Created on: Oct 23, 2025
 *      Author: vani.jr
 */

//SPI CONFIGURATION

// PIN PB13  SPI2_SCK
//PIN  PB15  SPI2_MOSI
//PIN PB14   SPI2_MISO
//PIN  PB12  SPI2_NSS

#include "driver_stm32f407xx.h"
#include<string.h>


extern void initialise_monitor_handles(void);
#define HIGH 1
#define BTN_PRESSED HIGH


#define COMMAND_LED_CTRL     0x50
#define COMMAND_SENSOR_READ  0x51
#define  COMMAND_LED_READ    0X52
#define COMMAND_PRINT        0x53
#define COMMAND_ID_READ      0x54

#define  LED_ON            1
#define  LED_OFF           0

#define ANALOG_PIN0        0
#define ANALOG_PIN1        1
#define ANALOG_PIN2        2
#define ANALOG_PIN3        3
#define ANALOG_PIN4        4

#define LED_PIN  8


void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
void SPI_GPIOInits(void){

	GPIO_Handle_t  SPIPins;
	SPIPins.pGPIOx  =GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl =GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed =GPIO_SPEED_FAST;


	//  SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
		GPIO_Init(&SPIPins);

    //MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
				GPIO_Init(&SPIPins);


   //NSS

		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
						GPIO_Init(&SPIPins);

}

void SPI_Inits(void){

	SPI_Handle_t SPIhandle ;

	SPIhandle.pSPIx = SPI2;

	SPIhandle.SPIConfig.SPI_BusConfig =SPI_BUS_CONFIG_FD;
	SPIhandle.SPIConfig.SPI_SSM  =SPI_SSM_DISABLE;
	SPIhandle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;

	SPIhandle.SPIConfig.SPI_SclkSpeed =SPI_SLK_SPEED_DIV8;
	SPIhandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;

	SPIhandle.SPIConfig.SPI_CPOL= SPI_CPOL_LOW;
	SPIhandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&SPIhandle);
}


void  SPI_BUTTONInits(){

	GPIO_Handle_t GpioLed,GpioBtn;

		GpioLed.pGPIOx =GPIOD;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


		GPIO_PeriClockControl(GPIOD,ENABLE);
		GPIO_Init(&GpioLed);


		GpioBtn.pGPIOx =GPIOA;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;


			GPIO_PeriClockControl(GPIOA,ENABLE);
			GPIO_Init(&GpioBtn);


}

uint8_t SPI_VerifyResponse(uint8_t  ackbyte){

	if(ackbyte == 0xF5){
		return 1;
	}

	return 0;

}
int main(void){



	uint8_t  dummy_write = 0xff;
	uint8_t  dummy_read;
	initialise_monitor_handles();

    printf("I am entered....\n");

	SPI_BUTTONInits();
	SPI_GPIOInits();
	SPI_Inits();
	SPI_SSOEConfig(SPI2,ENABLE);


	//to avoid modf error ,internally make the bit clear
   //	SPI_SSIConfig(SPI2, ENABLE);




while(1){

	   if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)==BTN_PRESSED){

	   	    printf(" after %d\n",GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
            delay();


	       SPI_PeripheralControl(SPI2,ENABLE);



//	       1)CMD_LED_CTRL    <PIN NO>  <VALUE>

	       uint8_t   commandcode   = COMMAND_LED_CTRL;
	        uint8_t ackbyte;
	        uint8_t args[2];
//
//	       SPI_SendData(SPI2,&commandcode,1);
//	       //after sendata the rxne is set so to remove the set value to reset
//
//	       SPI_ReceiveData(SPI2,&dummy_read,1);
//
//	// send some dummy byte to fetch the response from the slave , in shift register we have data ready so we need to move the data from the shift register for that pass the dummy data
//
//	       SPI_SendData(SPI2,&dummy_write,1);
//	       SPI_ReceiveData(SPI2,&ackbyte,1);
//
//
//	if( SPI_VerifyResponse(ackbyte)){
//		args[0]  =LED_PIN;
//		args[1]  =LED_ON;
//	       SPI_SendData(SPI2,args,2);
//
//	}
//
//
//
//
////	2. CMD_SENSOR_READ < analog pin number >
//
//	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
//
//     delay();
//
//
//
//     commandcode   = COMMAND_SENSOR_READ;
//
//
//    	       SPI_SendData(SPI2,&commandcode,1);
//    	       //after sendata the rxne is set so to remove the set value to reset
//
//    	       SPI_ReceiveData(SPI2,&dummy_read,1);
//
//    	// send some dummy byte to fetch the response from the slave , in shift register we have data ready so we need to move the data from the shift register for that pass the dummy data
//
//    	       SPI_SendData(SPI2,&dummy_write,1);
//    	       SPI_ReceiveData(SPI2,&ackbyte,1);
//
//
//    	if( SPI_VerifyResponse(ackbyte)){
//    		args[0]  = ANALOG_PIN0;
////    		args[1]  =LED_ON;
//    	       SPI_SendData(SPI2,args,1);
//
//    	}
//
//    	SPI_ReceiveData(SPI2,&dummy_read,1);
//
//	       //delay
//	       delay();
//
//	       SPI_SendData(SPI2,&dummy_write,1);
//
//
//    	uint8_t analog_read;
//    	SPI_ReceiveData(SPI2,&analog_read,1);
//
//
//    	printf("analog value %d\n",analog_read);
//
//
////   3. COMMAND LED READ
//
//    	     while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
//
//    	     delay();
//
//    	     commandcode   = COMMAND_LED_READ;
//
//
//    	     	       SPI_SendData(SPI2,&commandcode,1);
//    	     	       //after sendata the rxne is set so to remove the set value to reset
//
//    	     	      SPI_ReceiveData(SPI2,&dummy_read,1);
//
//    	     	// send some dummy byte to fetch the response from the slave , in shift register we have data ready so we need to move the data from the shift register for that pass the dummy data
//
//    	     	       SPI_SendData(SPI2,&dummy_write,1);
//    	     	      SPI_ReceiveData(SPI2,&ackbyte,1);
//
//
//    	     	if( SPI_VerifyResponse(ackbyte)){
//    	     		args[0]  =LED_PIN;
////    	     		args[1]  =LED_ON;
//    	     	       SPI_SendData(SPI2,args,1);
//    	     	      SPI_ReceiveData(SPI2,&dummy_read,1);
//                        delay();
//     	     	       SPI_SendData(SPI2,&dummy_write,1);
//
//     	     	       uint8_t LedRead;
//     	     	     SPI_ReceiveData(SPI2,&LedRead,1);
//
//   printf("Led Read %d\n",LedRead);
//    	     	}
//     4. COMMAND PRINT

    	                	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

    	     	    	     delay();

    	     	    	     commandcode   = COMMAND_PRINT;


    	     	    	     	       SPI_SendData(SPI2,&commandcode,1);
    	     	    	     	       //after sendata the rxne is set so to remove the set value to reset

    	     	    	     	     SPI_ReceiveData(SPI2,&dummy_read,1);

    	     	    	     	// send some dummy byte to fetch the response from the slave , in shift register we have data ready so we need to move the data from the shift register for that pass the dummy data

    	     	    	     	       SPI_SendData(SPI2,&dummy_write,1);
    	     	    	     	     SPI_ReceiveData(SPI2,&ackbyte,1);

                                 uint8_t message[]="hi vani";
    	     	    	     	if( SPI_VerifyResponse(ackbyte)){
    	     	    	     		args[0]  =strlen((char*)message);
    	     	//    	     		args[1]  =LED_ON;
    	     	    	     	       SPI_SendData(SPI2,args,1);
    	     	    	     	     SPI_ReceiveData(SPI2,&dummy_read,1);
    	     	                        delay();
//    	     	     	     	       SPI_SendData(SPI2,&dummy_write,1);

    	     	     	     	       for(int i=0;i<args[0];i++){
    	     	    	     	       SPI_SendData(SPI2,&message[i],1);
    	     	    	     	     SPI_ReceiveData(SPI2,&dummy_read,1);


    	     	     	     	       }


    	     	    	     	}

          //5.  CMD_ID_READ

    	     	    	        	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

    	     	    	       	     delay();

    	     	    	       	     commandcode   = COMMAND_ID_READ;


    	     	    	       	     	       SPI_SendData(SPI2,&commandcode,1);
    	     	    	       	     	       //after sendata the rxne is set so to remove the set value to reset

    	     	    	       	     	  SPI_ReceiveData(SPI2,&dummy_read,1);

    	     	    	       	     	// send some dummy byte to fetch the response from the slave , in shift register we have data ready so we need to move the data from the shift register for that pass the dummy data

    	     	    	       	     	       SPI_SendData(SPI2,&dummy_write,1);
    	     	    	       	     	  SPI_ReceiveData(SPI2,&ackbyte,1);

    	     	    	       		       uint8_t IR_READ[11];
    	     	    	       		       uint8_t i=0;
    	     	    	       	     	if( SPI_VerifyResponse(ackbyte)){
//
    	     	    	       	     		for(int i=0;i<10;i++){
    	     	    	        	     	       SPI_SendData(SPI2,&dummy_write,1);
    	     	    	        	     	     SPI_ReceiveData(SPI2,&IR_READ[i],1);

    	     	    	       	     		}
    	     	    	       	     		IR_READ[i]='\0';
    	     	    	       	     	}


    	     	    	       	     	for(int i=0;i<11;i++){

    	     	    	       	     	printf("IR read  %d\n",IR_READ[i]);}




//
//	uint8_t dataLen =  strlen(user_data);
//	SPI_SendData(SPI2,&dataLen,1);
//
//	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	while( SPI_GetFlagStatus(SPI2,SPI_BSY_FLAG) );

	SPI_PeripheralControl(SPI2, DISABLE);
	   }
}
	return 0;
}
