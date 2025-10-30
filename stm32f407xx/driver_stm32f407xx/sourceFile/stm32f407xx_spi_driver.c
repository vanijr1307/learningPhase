/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 21, 2025
 *      Author: vani.jr
 */


#include "stm32f407xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);  //here to make this function as private in this file
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{

	if(EnorDi ==ENABLE){

			if(pSPIx==SPI1){
				SPI1_PCLK_EN();
			}else if(pSPIx==SPI2){
				SPI2_PCLK_EN();
			}
			else if(pSPIx==SPI3){
				SPI3_PCLK_EN();
					}
			else if(pSPIx==SPI4){
				SPI4_PCLK_EN();
			}
		}else{

			if(pSPIx==SPI1){
				SPI1_PCLK_DI();
					}else if(pSPIx==SPI2){
						SPI2_PCLK_DI();
					}
					else if(pSPIx==SPI3){
						SPI3_PCLK_DI();
							}
					else if(pSPIx==SPI4){
						SPI4_PCLK_DI();
							}


		}

}

void SPI_Init(SPI_Handle_t *pSPIHandle){

	uint32_t tempreg =0;
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD){

		tempreg &=  ~(1<<SPI_CR1_BIDI_MODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD){
		tempreg |=  (1<<SPI_CR1_BIDI_MODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		tempreg &=  ~(1<<SPI_CR1_BIDI_MODE);
        tempreg  |=(1<<SPI_CR1_RX);
	}

	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;
	tempreg  |= pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF;
	tempreg  |= pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;
	tempreg  |=pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 =tempreg;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx==SPI1){
		SPI1_REG_RESET();
			}else if(pSPIx==SPI2){
				SPI2_REG_RESET();
			}
			else if(pSPIx==SPI3){
				SPI3_REG_RESET();
					}
			else if(pSPIx==SPI4){
				SPI4_REG_RESET();
					}

}



uint8_t  SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName){

	if(pSPIx->SR & FlagName)  return FLAG_SET;

	return FLAG_RESET;
}
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t length)
{  //BLOCKING API   WAIT UNTIL ALL BITS ARE TRANSFERED
	//FIRMWARE HAVE NOT DIRECT ACCESS TO TENASMITTER AND RECEIVER DO IT BY DATA REGISTER COPIED INTO DR AND CPOIED TO THE TX BUFFER AND RX BUFFER
	while(length>0){


		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) ==FLAG_RESET);


		if(pSPIx->CR1 & (1<<SPI_CR1_DFF)){


			pSPIx->DR =*((uint16_t *)pTxBuffer);
			length-=2;
			(uint16_t *)pTxBuffer++;
		}else{
			pSPIx->DR =*(pTxBuffer);
						length--;
						pTxBuffer++;
		}



	}


}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t length){

	while(length>0){


			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) ==FLAG_RESET);


			if(pSPIx->CR1 & (1<<SPI_CR1_DFF)){


				*((uint16_t *)pRxBuffer)  =pSPIx->DR ;
				length-=2;
				(uint16_t *)pRxBuffer++;
			}else{
				*(pRxBuffer)  =pSPIx->DR ;
							length--;
							pRxBuffer++;
			}



		}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *SPIHandle,uint8_t *pTxBuffer,uint32_t length)
{
	uint8_t state =SPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){

	//1. Save the Tx buffer  address and Len information in some global variables.

	SPIHandle->pTxBuffer=pTxBuffer;
	SPIHandle->TxLen=length;


	//2. Mark the spi state as busy in transmission so that no other code can take over same spi peripheral until transmission is over.

	SPIHandle->TxState  =SPI_BUSY_IN_TX;


	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set SR.
	SPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);

	//4. Data Transmission will be handled by the ISR code



	}

	return state;


}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *SPIHandle,uint8_t *pRxBuffer,uint32_t length){

	uint8_t state =SPIHandle->RxState;
		if(state != SPI_BUSY_IN_RX){

		//1. Save the Tx buffer  address and Len information in some global variables.

		SPIHandle->pRxBuffer=pRxBuffer;
		SPIHandle->RxLen=length;


		//2. Mark the spi state as busy in transmission so that no other code can take over same spi peripheral until transmission is over.

		SPIHandle->RxState  =SPI_BUSY_IN_RX;


		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set SR.
		SPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);

		//4. Data Transmission will be handled by the ISR code



		}

		return state;





}








void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(IRQNumber <=31){
			//program ISER0 register
            *NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <64){
			//ISER1 register
            *NVIC_ISER1 |= (1<<(IRQNumber%32));

		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
					//ISER2 register
            *NVIC_ISER2 |= (1<<(IRQNumber%64));

				}
	}
	else{
		if(IRQNumber <=31){
				//program IcER0 register

			*NVIC_ICER0 |= (1<<IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber <64){
				//IcER1 register
				*NVIC_ICER0 |= (1<<(IRQNumber%32));
			}
			else if(IRQNumber >= 64 && IRQNumber < 96){
						//IcER2 register
				*NVIC_ICER0 |= (1<<(IRQNumber%64));
					}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

	uint8_t  iprx  =IRQNumber /4;
	uint8_t  iprx_section = IRQNumber%4;
	uint8_t  shift_amount  =(8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx )|= (IRQPriority << shift_amount);   //  *(a+n)=>a[n]  n denotes the (no of place * return type value)    ex:  *(100 +5)=> 100 is 2 bits means *(100+(2*5))
}



void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
     // 1. ENTER ISR
	//2. UNDERSTAND WHICH EVENT CAUSED INTERRUPT TO TRIGGER (CHECKS SR STATUS REGISTER)
	//3. INTERRUPT DUE TO SETTING OF RXNE FLAG ->HANDLE RXNE EVENT
	//OR  INTERRUPT DUE TO SETTING OF TXE FLAG ->HANDLE TXE FLAG
	// OR INTERRUPT IS DUE SETTING OF ERROR FLAG -> HANDLE ERROR
	uint8_t temp1,temp2;

	temp1 = pHandle->pSPIx->SR &(1<< SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 &(1<< SPI_CR2_TXEIE);


	if(temp1 && temp2){
		spi_txe_interrupt_handle(pHandle);

	}

	    temp1 = pHandle->pSPIx->SR &(1<< SPI_SR_RXNE);
		temp2 = pHandle->pSPIx->CR2 &(1<< SPI_CR2_RXNEIE);


		if(temp1 && temp2){
			spi_rxne_interrupt_handle(pHandle);

		}
	        temp1 = pHandle->pSPIx->SR &(1<< SPI_SR_OVR);
			temp2 = pHandle->pSPIx->CR2 &(1<< SPI_CR2_ERRIE);


			if(temp1 && temp2){
				spi_ovr_err_interrupt_handle(pHandle);

			}
}
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		pSPIx->CR1 |=  (1<< SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1<< SPI_CR1_SPE);
	}

}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		pSPIx->CR1 |=  (1<< SPI_CR1_SSI);
	}else
	{
		pSPIx-> CR1 &=  ~(1<< SPI_CR1_SSI);
	}

}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		pSPIx->CR2 |=  (1<< SPI_CR2_SSOE);
	}else
	{
		pSPIx-> CR2 &=  ~(1<< SPI_CR2_SSOE);
	}

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF)){


		pSPIHandle->pSPIx->DR =*((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -=2;
			(uint16_t *)pSPIHandle->pTxBuffer++;
		}else{
			pSPIHandle->pSPIx->DR =*(pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;

}

	if(! pSPIHandle->TxLen){
//		txlen  is zero, so close the spi transmission and inform the application that tx is over.
//		this prevent interrupts from setting up of txe flag
//		pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_TXEIE);
//		pSPIHandle->pTxBuffer =NULL;
//		pSPIHandle->TxLen =0;
//		pSPIHandle->TxState =SPI_READY;

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_COMPLT);
	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){    // after the isr complete which  is enabled from senddataIT txeie is set
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF)){


			*((uint16_t *)pSPIHandle->pRxBuffer)  =pSPIHandle->pSPIx->DR ;
			pSPIHandle->RxLen -=2;
				(uint16_t *)pSPIHandle->pRxBuffer++;
			}else{
				*(pSPIHandle->pRxBuffer)=pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer++;

	}

		if(! pSPIHandle->RxLen){
	//		txlen  is zero, so close the spi transmission and inform the application that tx is over.
	//		this prevent interrupts from setting up of txe flag
//			pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_RXNEIE);
//			pSPIHandle->pRxBuffer =NULL;
//			pSPIHandle->RxLen =0;
//			pSPIHandle->RxState =SPI_READY;

			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_COMPLT);
		}

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;

	if(pSPIHandle->TxState!=  SPI_BUSY_IN_TX){

		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}

	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);


}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx -> DR;
	temp = pSPIx-> SR;
	(void)temp;

}
void SPI_CloseTransmission(SPI_Handle_t *SPIHandle){
	        pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_TXEIE);
			pSPIHandle->pTxBuffer =NULL;
			pSPIHandle->TxLen =0;
			pSPIHandle->TxState =SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *SPIHandle){
				pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_RXNEIE);
				pSPIHandle->pRxBuffer =NULL;
				pSPIHandle->RxLen =0;
				pSPIHandle->RxState =SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *SPIHandle,uint8_t AppEvt){

}



