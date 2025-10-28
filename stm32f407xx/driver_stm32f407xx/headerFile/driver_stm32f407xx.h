/*
 * driver_stm32f407xx.h
 *
 *  Created on: Oct 15, 2025
 *      Author: vani.jr
 */

#ifndef INC_DRIVER_STM32F407XX_H_
#define INC_DRIVER_STM32F407XX_H_
#include <stdint.h>
#include<stdio.h>
#include<stddef.h>
#define vo volatile

#define NVIC_ISER0  ((vo uint32_t *)0xE000E100)
#define NVIC_ISER1  ((vo uint32_t *)0xE000E104)
#define NVIC_ISER2   ((vo uint32_t *)0xE000E108)
#define NVIC_ISER3  ((vo uint32_t *)0xE000E10c)

#define NVIC_ICER0  ((vo uint32_t *)0xE000E180)
#define NVIC_ICER1  ((vo uint32_t *)0xE000E184)
#define NVIC_ICER2   ((vo uint32_t *)0xE000E188)
#define NVIC_ICER3  ((vo uint32_t *)0xE000E18c)


#define NVIC_PR_BASE_ADDR  (( vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED     4


#define FLASH_BASEADDR            0x08000000U   // macro dont use memory
#define SRAM                     SRAM1_BASEADDR
#define SRAM1_BASEADDR           0x20000000U   //112KB
#define SRAM2_BASEADDR           0x2001C000U   //16KB
#define ROM                      0x1FFF0000U    //EMBEDDED FLASH MEMORY IN TABLE 5. SYSTEM MEMORY
//#define SRAM                     SRAM1_BASEADDR

//BASED ON DATA COMMUNICATION BUS INERFACES ARE USED
#define PERIPH_BASEADDR           0x40000000U
#define APB1PERIPH_BASEADDR       PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR        0x40010000U
#define AHB1PERIPH_BASEADDR        0x40020000U
#define AHB2PERIPH_BASEADDR         0x50000000U


//GPIO PORTS WITH THE BASE ADDRESS + OFFSET AHB
#define GPIOA_BASEADDR       (AHB1PERIPH_BASEADDR+0X0000)  //there no difference between 0xabc and 0XABC and 0Xabc and 0xABC
#define GPIOB_BASEADDR       (AHB1PERIPH_BASEADDR+0X0400)
#define GPIOC_BASEADDR       (AHB1PERIPH_BASEADDR+0X0800)
#define GPIOD_BASEADDR       (AHB1PERIPH_BASEADDR+0X0C00)
#define GPIOE_BASEADDR       (AHB1PERIPH_BASEADDR+0X1000)
#define GPIOF_BASEADDR       (AHB1PERIPH_BASEADDR+0X1400)
#define GPIOG_BASEADDR       (AHB1PERIPH_BASEADDR+0X1800)
#define GPIOH_BASEADDR       (AHB1PERIPH_BASEADDR+0X1C00)
#define GPIOI_BASEADDR       (AHB1PERIPH_BASEADDR+0X2000)
//#define GPIOJ_BASEADDR       (AHB1PERIPH_BASEADDR+0X2400)
//#define GPIOK_BASEADDR       (AHB1PERIPH_BASEADDR+0X2800)
#define RCC_BASEADDR       (AHB1PERIPH_BASEADDR+0X3800)

//add pheripheral for APB1
#define I2C1_BASEADDR              (APB1PERIPH_BASEADDR +0X5400)
#define I2C2_BASEADDR              (APB1PERIPH_BASEADDR +0X5800 )
#define I2C3_BASEADDR              (APB1PERIPH_BASEADDR +0X5C00)
#define SPI2_BASEADDR              (APB1PERIPH_BASEADDR +0X3800)
#define SPI3_BASEADDR              (APB1PERIPH_BASEADDR +0X3C00)
#define USART2_BASEADDR             (APB1PERIPH_BASEADDR +0X4400)
#define USART3_BASEADDR             (APB1PERIPH_BASEADDR +0X4800)
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR +0X4C00)
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR +0X5000)

//ADD pheripheral for APB2
#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0X3000 )
#define SPI4_BASEADDR               (APB2PERIPH_BASEADDR + 0X3400 )

#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0X1000 )
#define USART6_BASEADDR             (APB2PERIPH_BASEADDR + 0X1400 )
#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0X3C00 )
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0X3800 )


typedef struct{
	vo uint32_t CR1;
	vo uint32_t CR2;
	vo uint32_t  SR;
	vo uint32_t   DR;
	vo uint32_t  CRCPR;
	vo uint32_t  RXCRCR;
	vo uint32_t  TXCRCR;
	vo uint32_t  I2SCFGR;
	vo uint32_t  I2SPR;

}SPI_RegDef_t;

typedef struct
{
	vo uint32_t  MODER ;
	vo uint32_t OTYPER;
	vo uint32_t  OSPEEDR;
	vo uint32_t PUPDR;
	vo uint32_t IDR ;
	vo uint32_t ODR;
	vo uint32_t BSRR ;
	vo uint32_t LCKR;
	vo uint32_t AFR[2];


}GPIO_RegDef_t;

typedef struct
{
	vo uint32_t  CR;
	vo uint32_t  PLLCFGR;
     vo uint32_t  CFGR;
	vo uint32_t  CIR;
	vo uint32_t  AHB1RSTR;
	vo uint32_t  AHB2RSTR;
	vo uint32_t  AHB3RSTR;
	 uint32_t  RESERVED0;
	vo uint32_t  APB1RSTR;
	vo uint32_t  APB2RSTR;
	uint32_t  RESERVED1[2];   //  0x28,0x2C
	vo uint32_t   AHB1ENR;
	vo uint32_t   AHB2ENR;
	vo uint32_t   AHB3ENR;
	vo uint32_t   RESERVED2;
	vo uint32_t   APB1ENR;
	vo uint32_t   APB2ENR;
	uint32_t  RESERVED3[2];  // 0x48, 0x4C
	vo uint32_t  AHB1LPENR;
	vo uint32_t   AHB2LPENR;
	vo uint32_t    AHB3LPENR;
	uint32_t    RESERVED4;
	vo uint32_t     APB1LPENR;
	vo uint32_t      APB2LPENR;
	uint32_t     RESERVED5[2];  // 0x68, 0x6C
	vo uint32_t     BDCR;
	vo uint32_t     CSR;
	uint32_t     RESERVED6[2];   // 0x78, 0x7C
	vo uint32_t     SSCGR;
	vo uint32_t     PLLI2SCFGR;
	vo uint32_t     PLLSAICFGR;
	vo uint32_t     DCKACFGR;
	vo uint32_t     CKGATENR;
	vo uint32_t     DCKCFGR2;


}RCC_RegDef_t;

typedef struct
{
 vo uint32_t  IMR;
 vo uint32_t  EMR;
 vo uint32_t  RTSR;
 vo uint32_t  FTSR;
 vo uint32_t  SWIER;
 vo uint32_t  PR;
}EXTI_RegDef_t;


typedef struct
{  vo uint32_t  MEMRMP;
   vo uint32_t  PMC;
   vo uint32_t  EXTICR[4];
    uint32_t  RESERVED1[2];
   vo uint32_t  CMPCR;
    uint32_t  RESERVED2[2];
    vo uint32_t  CFGR;



}SYSCFG_RegDef_t;

#define GPIOA      ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB      ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC      ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD      ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE      ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF      ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG      ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH     ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI      ((GPIO_RegDef_t *)GPIOI_BASEADDR)



#define RCC      ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI      ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG    ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1        ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2        ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3        ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4        ((SPI_RegDef_t*)SPI4_BASEADDR)



#define GPIOA_PCLCK_EN()  RCC->AHB1ENR |= (1<<0)
#define GPIOB_PCLCK_EN()  RCC->AHB1ENR |= (1<<1)
#define GPIOC_PCLCK_EN()  RCC->AHB1ENR |= (1<<2)
#define GPIOD_PCLCK_EN()  RCC->AHB1ENR |= (1<<3)
#define GPIOE_PCLCK_EN()  RCC->AHB1ENR |= (1<<4)
#define GPIOF_PCLCK_EN()  RCC->AHB1ENR |= (1<<5)
#define GPIOG_PCLCK_EN()  RCC->AHB1ENR |= (1<<6)
#define GPIOH_PCLCK_EN()  RCC->AHB1ENR |= (1<<7)
#define GPIOI_PCLCK_EN()  RCC->AHB1ENR |= (1<<8)

#define GPIOA_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLCK_DI()  (RCC->AHB1ENR &= ~(1<<8))

#define I2C1_PCLK_EN()   RCC->APB1ENR |= (1<<21)
#define I2C2_PCLK_EN()   RCC->APB1ENR |= (1<<22)
#define I2C3_PCLK_EN()   RCC->APB1ENR |= (1<<23)
#define SPI2_PCLK_EN()   RCC->APB1ENR |= (1<<14)
#define SPI3_PCLK_EN()   RCC->APB1ENR |= (1<<15)
#define USART2_PCLK_EN() RCC->APB1ENR |= (1<<17)
#define USART3_PCLK_EN() RCC->APB1ENR |= (1<<18)
#define UART4_PCLK_EN()  RCC->APB1ENR |= (1<<19)
#define UART5_PCLK_EN()  RCC->APB1ENR |= (1<<20)

#define I2C1_PCLK_DI()  ( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()  ( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()  ( RCC->APB1ENR &= ~(1<<23) )
#define SPI2_PCLK_DI()   ( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()   ( RCC->APB1ENR &= ~(1<<15) )
#define USART2_PCLK_DI() ( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI() ( RCC->APB1ENR &= ~(1<<18) )
#define UART4_PCLK_DI()  (RCC->APB1ENR &= ~(1<<19) )
#define UART5_PCLK_DI()  (RCC->APB1ENR &= ~(1<<20) )

#define SPI1_PCLK_EN()       RCC->APB2ENR |= (1<<12)
#define SPI4_PCLK_EN()       RCC->APB2ENR |= (1<<13)

#define USART1_PCLK_EN()     RCC->APB2ENR |= (1<<4)
#define USART6_PCLK_EN()     RCC->APB2ENR |= (1<<5)

#define SYSCFG_PCLK_EN()     RCC->APB2ENR |= (1<<14)

#define SPI1_PCLK_DI()       (RCC->APB2ENR &= ~(1<<12))
#define SPI4_PCLK_DI()       (RCC->APB2ENR &= ~(1<<13))

#define USART1_PCLK_DI()    ( RCC->APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI()    ( RCC->APB2ENR &= ~(1<<5))
#define SYSCFG_PCLK_DI()    ( RCC->APB2ENR &= ~(1<<14))

#define GPIOA_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)

#define SPI1_REG_RESET()    do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()    do{ (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()    do{ (RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()    do{ (RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)      ((x==GPIOA)? 0:\
		                               (x==GPIOB)? 1:\
                                       (x==GPIOC)? 2:\
		                                (x==GPIOD)?3:\
				                        (x==GPIOE)? 4:\
						                (x==GPIOF)? 5:\
								         (x==GPIOG)? 6:\
								        (x==GPIOH)? 7:8	 )





#define IRQ_NO_EXTI0     6
#define IRQ_NO_EXTI1     7
#define IRQ_NO_EXTI2     8
#define IRQ_NO_EXTI3     9
#define IRQ_NO_EXTI4     10
#define IRQ_NO_EXTI9_5   23
#define IRQ_NO_EXTI15_10  40
#define IRQ_NO_SPI1       35
#define IRQ_NO_SPI2       36
#define IRQ_NO_SPI3       51




#define NVIC_IRQ_PRI0     0
#define NVIC_IRQ_PRI15    15

#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET RESET
#define  FLAG_RESET    RESET
#define  FLAG_SET      SET




//spi position

#define SPI_CR1_CPHA   0
#define SPI_CR1_CPOL   1
#define SPI_CR1_MSTR   2
#define SPI_CR1_BR     3
#define SPI_CR1_SPE    6
#define SPI_CR1_LSBFIRST   7
#define SPI_CR1_SSI        8
#define SPI_CR1_SSM       9
#define SPI_CR1_RX   10
#define SPI_CR1_DFF   11
#define SPI_CR1_CRCNEXT   12
#define SPI_CR1_CRCEN   13
#define SPI_CR1_BIDI_OE  14
#define SPI_CR1_BIDI_MODE   15


#define SPI_CR2_RXDMAEN  0
#define SPI_CR2_TXDMAEN  1
#define SPI_CR2_SSOE  2
#define SPI_CR2_FRF  4
#define SPI_CR2_ERRIE  5
#define SPI_CR2_RXNEIE  6
#define SPI_CR2_TXEIE  7


#define SPI_SR_RXNE  0
#define SPI_SR_TXE  1
#define SPI_SR_CHSIDE  2
#define SPI_SR_UDR  3
#define SPI_SR_CRCERR  4
#define SPI_SR_MODF  5
#define SPI_SR_OVR  6
#define SPI_SR_BSY  7
#define SPI_SR_FRE  8


#define SPI_TXE_FLAG   (1<< SPI_SR_TXE)
#define SPI_RXNE_FLAG   (1<< SPI_SR_RXNE)
#define SPI_BSY_FLAG   (1<< SPI_SR_BSY)


#include  "stm32f407xx_gpio_driver.h"
#include  "stm32f407xx_spi_driver.h"

#endif /* INC_DRIVER_STM32F407XX_H_ */
