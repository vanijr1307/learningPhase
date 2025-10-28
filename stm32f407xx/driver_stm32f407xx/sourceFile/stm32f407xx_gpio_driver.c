/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 16, 2025
 *      Author: vani.jr
 */


#include "stm32f407xx_gpio_driver.h"


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){


	if(EnorDi ==ENABLE){

		if(pGPIOx==GPIOA){
			GPIOA_PCLCK_EN();
		}else if(pGPIOx==GPIOB){
			GPIOB_PCLCK_EN();
		}
		else if(pGPIOx==GPIOC){
					GPIOC_PCLCK_EN();
				}
		else if(pGPIOx==GPIOD){
					GPIOD_PCLCK_EN();
				}
		else if(pGPIOx==GPIOE){
					GPIOE_PCLCK_EN();
				}
		else if(pGPIOx==GPIOF){
					GPIOF_PCLCK_EN();
				}
		else if(pGPIOx==GPIOG){
					GPIOG_PCLCK_EN();
				}
		else if(pGPIOx==GPIOH){
					GPIOH_PCLCK_EN();
				}
		else if(pGPIOx==GPIOI){
					GPIOI_PCLCK_EN();
				}
	}else{

		if(pGPIOx==GPIOA){
					GPIOA_PCLCK_DI();
				}else if(pGPIOx==GPIOB){
					GPIOB_PCLCK_DI();
				}
				else if(pGPIOx==GPIOC){
							GPIOC_PCLCK_DI();
						}
				else if(pGPIOx==GPIOD){
							GPIOD_PCLCK_DI();
						}
				else if(pGPIOx==GPIOE){
							GPIOE_PCLCK_DI();
						}
				else if(pGPIOx==GPIOF){
							GPIOF_PCLCK_DI();
						}
				else if(pGPIOx==GPIOG){
							GPIOG_PCLCK_DI();
						}
				else if(pGPIOx==GPIOH){
							GPIOH_PCLCK_DI();
						}
				else if(pGPIOx==GPIOI){
							GPIOI_PCLCK_DI();
						}

	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t  temp=0;
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG){
		temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // pinnumber as 5 (2bit to define the mode ) so 2*5 =10,  )
		//collect the details in what type pin mode is coming here.
		pGPIOHandle->pGPIOx ->MODER &=  ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx ->MODER |=temp; //not using bitwise or | in that condition will affect the of other bit which make other work affected.


	}else{
      if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_FT)
      {
    	  EXTI->FTSR  |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	  EXTI->RTSR  &= ~(1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

      }
      else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_RT){
    	  EXTI->RTSR  |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	  EXTI->FTSR  &= ~(1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

      }
      else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_RFT){
    	  EXTI->FTSR  |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	  EXTI->RTSR  |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

      }

     //configure the GPIO Port selection in SYSCFG_EXTICR
      uint8_t temp1  =pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber/4;
      uint8_t temp2  =pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber%4;
      uint8_t portcode =GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
      SYSCFG_PCLK_EN();
      SYSCFG->EXTICR[temp1] = portcode << (temp2*4);

      //enable exti interrupt delivery using IMR

	  EXTI->IMR  |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp=0;
	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // pinnumber as 5 (2bit to define the mode ) so 2*5 =10,  )
	pGPIOHandle->pGPIOx ->OSPEEDR &=  ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx ->OSPEEDR |=temp;

	temp=0;
	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // pinnumber as 5 (2bit to define the mode ) so 2*5 =10,  )
	pGPIOHandle->pGPIOx ->PUPDR &=  ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx ->PUPDR |=temp;


		temp=0;
			temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // pinnumber as 5 (2bit to define the mode ) so 2*5 =10,  )
			pGPIOHandle->pGPIOx ->OTYPER &=  ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			pGPIOHandle->pGPIOx ->OTYPER |=temp;


		temp =0;

	if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
         uint8_t temp1,temp2;
         temp1 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
         temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
         pGPIOHandle->pGPIOx->AFR[temp1] &=~(0xF << (4*temp2));

         pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx==GPIOA){
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOB){
				GPIOA_REG_RESET();
			}
			else if(pGPIOx==GPIOC){
				GPIOA_REG_RESET();
					}
			else if(pGPIOx==GPIOD){
				GPIOA_REG_RESET();
					}
			else if(pGPIOx==GPIOE){
				GPIOA_REG_RESET();
					}
			else if(pGPIOx==GPIOF){
				GPIOA_REG_RESET();
					}
			else if(pGPIOx==GPIOG){
				GPIOA_REG_RESET();
					}
			else if(pGPIOx==GPIOH){
				GPIOA_REG_RESET();
					}
			else if(pGPIOx==GPIOI){
				GPIOA_REG_RESET();
					}
}


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value= (uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint8_t value;
		value= (uint16_t)(pGPIOx->IDR );
		return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value){

	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1<<PinNumber);
	}else{
		pGPIOx->ODR &= ~(1<<PinNumber);

	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value){
	pGPIOx->ODR =Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^=(1<<PinNumber);
}



void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){

	uint8_t  iprx  =IRQNumber /4;
	uint8_t  iprx_section = IRQNumber%4;
	uint8_t  shift_amount  =(8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx )|= (IRQPriority << shift_amount);   //  *(a+n)=>a[n]  n denotes the (no of place * return type value)    ex:  *(100 +5)=> 100 is 2 bits means *(100+(2*5))
}
void GPIO_IRQHandling(uint8_t PinNumber){

	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR &(1<<PinNumber)){       // enable when the falling or rising trigger happened
		//CLEAR TO WRITE ONE TO THE PR REGISTER
		EXTI->PR |=(1<<PinNumber);
	}
}
