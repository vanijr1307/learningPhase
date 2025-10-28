/*
 * 005_spi_sendData.c
 *
 *  Created on: Oct 22, 2025
 *      Author: vani.jr
 */



//  SPI Configuration

//PIN PA5   SPI1_SCK
//PIN PA6   SPI1_MISO
//PIN PA7   SPI1_MOSI
//PIN PA4   SPI2_NSS
// ALTERANTE function mode :5

//ALSO DONE FOR THE SPI2 PORT B
// PIN PB13  SPI2_SCK
//PIN  PB15  SPI2_MOSI
//PIN PB14   SPI2_MISO
//PIN  PB12  SPI2_NSS

#include "driver_stm32f407xx.h"
#include<string.h>

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
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
//		GPIO_Init(&SPIPins);

    //MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
				GPIO_Init(&SPIPins);
//
//				SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//				SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
//				SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
//				GPIO_Init(&SPIPins);


   //NSS

//		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
//						GPIO_Init(&SPIPins);

}

void SPI_Inits(void){

	SPI_Handle_t SPIhandle ;

	SPIhandle.pSPIx = SPI2;

	SPIhandle.SPIConfig.SPI_BusConfig =SPI_BUS_CONFIG_FD;
	SPIhandle.SPIConfig.SPI_SSM  =SPI_SSM_ENABLE;
	SPIhandle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;

	SPIhandle.SPIConfig.SPI_SclkSpeed =SPI_SLK_SPEED_DIV2;
	SPIhandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;

	SPIhandle.SPIConfig.SPI_CPOL= SPI_CPOL_LOW;
	SPIhandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&SPIhandle);
}

int main(void){
    char user_data[]="Hello VANI";
	SPI_GPIOInits();
	SPI_Inits();
	//to avoid modf error ,internally make the bit clear
	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2,ENABLE);


	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	while( SPI_GetFlagStatus(SPI2,SPI_BSY_FLAG) );

	SPI_PeripheralControl(SPI2, DISABLE);
	return 0;
}
