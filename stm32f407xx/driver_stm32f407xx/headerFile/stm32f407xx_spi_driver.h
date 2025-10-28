/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Oct 21, 2025
 *      Author: vani.jr
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
#include  "driver_stm32f407xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;  //master or slave
	uint8_t SPI_BusConfig;  //half duplex and simplex
	uint8_t SPI_SclkSpeed;   //serial clock speed
	uint8_t SPI_DFF;   //data frame format 8bit or 16 bit
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;   //hardware or software slave management
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t  *pSPIx;
	SPI_Config_t  SPIConfig;
	uint8_t       *pTxBuffer;
	uint8_t       *pRxBuffer;
	uint32_t        TxLen;
	uint32_t        RxLen;
	uint8_t         TxState;
	uint8_t         RxState;
}SPI_Handle_t;

#define  SPI_DEVICE_MODE_MASTER   1
#define  SPI_DEVICE_MODE_SLAVE    0


#define SPI_READY               0
#define SPI_BUSY_IN_RX          1
#define SPI_BUSY_IN_TX          2

#define SPI_EVENT_TX_COMPLT     1
#define SPI_EVENT_RX_COMPLT     2
#define  SPI_EVENT_OVR_ERR      3
#define  SPI_EVENT_CRC_ERR      4





#define   SPI_BUS_CONFIG_FD     1
#define   SPI_BUS_CONFIG_HD     2
#define   SPI_BUS_CONFIG_SIMPLEX_TXONLY   3
#define   SPI_BUS_CONFIG_SIMPLEX_RXONLY   4

#define SPI_SLK_SPEED_DIV2             0
#define SPI_SLK_SPEED_DIV4             1
#define SPI_SLK_SPEED_DIV8             2
#define SPI_SLK_SPEED_DIV16            3
#define SPI_SLK_SPEED_DIV32            4
#define SPI_SLK_SPEED_DIV64            5
#define SPI_SLK_SPEED_DIV128           6
#define SPI_SLK_SPEED_DIV256           7


#define SPI_DFF_8BITS    0
#define SPI_DFF_16BITS   1

#define  SPI_CPOL_LOW   0
#define  SPI_CPOL_HIGH  1

#define  SPI_CPHA_LOW   0
#define  SPI_CPHA_HIGH  1

#define  SPI_SSM_ENABLE    1
#define  SPI_SSM_DISABLE    0


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t length);

uint8_t SPI_SendDataIT(SPI_Handle_t *SPIHandle,uint8_t *pTxBuffer,uint32_t length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *SPIHandle,uint8_t *pRxBuffer,uint32_t length);


void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *SPIHandle);
void SPI_CloseReception(SPI_Handle_t *SPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *SPIHandle,uint8_t AppEvt);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
