/*
 * stm32f103c8.h
 *
 *  Created on: Jun 29, 2020
 *      Author: Dave
 */

#ifndef INC_STM32F103C8_H_
#define INC_STM32F103C8_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/********************************** [	PROCESSOR SPECIFIC DETAILS	] **********************************
 *
 * ARM Cortex-M3 Processor NVIC ISERx Register Addresses
 */

//	Interrupt Set Registers
#define CPU_NVIC_ISER0					( (__vo uint32_t*) 0xE000E100 )
#define CPU_NVIC_ISER1					( (__vo uint32_t*) 0xE000E104 )
#define CPU_NVIC_ISER2					( (__vo uint32_t*) 0xE000E108 )
#define CPU_NVIC_ISER3					( (__vo uint32_t*) 0xE000E10C )
#define CPU_NVIC_ISER4					( (__vo uint32_t*) 0xE000E110 )
#define CPU_NVIC_ISER5					( (__vo uint32_t*) 0xE000E114 )
#define CPU_NVIC_ISER6					( (__vo uint32_t*) 0xE000E118 )
#define CPU_NVIC_ISER7					( (__vo uint32_t*) 0xE000E11C )

//	Interrupt Clear Registers
#define CPU_NVIC_ICER0					( (__vo uint32_t*) 0XE000E180 )
#define CPU_NVIC_ICER1					( (__vo uint32_t*) 0XE000E184 )
#define CPU_NVIC_ICER2					( (__vo uint32_t*) 0XE000E188 )
#define CPU_NVIC_ICER3					( (__vo uint32_t*) 0XE000E18C )
#define CPU_NVIC_ICER4					( (__vo uint32_t*) 0XE000E190 )
#define CPU_NVIC_ICER5					( (__vo uint32_t*) 0XE000E194 )
#define CPU_NVIC_ICER6					( (__vo uint32_t*) 0XE000E198 )
#define CPU_NVIC_ICER7					( (__vo uint32_t*) 0XE000E19C )

//	Interrupt Priority Registers
#define CPU_MVIC_IPR_BASE				( (__vo uint32_t*) 0xE000E400 )
#define NO_PR_BITS_IMPLEMENTED			4

//******************************************************************************************************

//	IRQ (Interrupt Request) Number Position of STM32F303xx MCU
#define IRQn_EXTI0							6
#define IRQn_EXTI1							7
#define IRQn_EXTI2							8
#define IRQn_EXTI3							9
#define IRQn_EXTI4							10
#define IRQn_EXTI9_5						23
#define IRQn_I2C1_EV						31
#define IRQn_I2C1_ER						32
#define IRQn_I2C2_EV						33
#define IRQn_I2C2_ER						34
#define IRQn_SPI1							35
#define IRQn_SPI2							36
#define IRQn_USART1							37
#define IRQn_USART2							38
#define IRQn_USART3							39
#define IRQn_EXTI15_10						40


// IRQ Priorities
#define NVIC_IRQ_PRIORITY_DEFAULT			0
#define NVIC_IRQ_PRIORITY_1					1
#define NVIC_IRQ_PRIORITY_2					2
#define NVIC_IRQ_PRIORITY_3					3
#define NVIC_IRQ_PRIORITY_4					4
#define NVIC_IRQ_PRIORITY_5					5
#define NVIC_IRQ_PRIORITY_6					6
#define NVIC_IRQ_PRIORITY_7					7
#define NVIC_IRQ_PRIORITY_8					8
#define NVIC_IRQ_PRIORITY_9					9
#define NVIC_IRQ_PRIORITY_10				10
#define NVIC_IRQ_PRIORITY_11				11
#define NVIC_IRQ_PRIORITY_12				12
#define NVIC_IRQ_PRIORITY_13				13
#define NVIC_IRQ_PRIORITY_14				14
#define NVIC_IRQ_PRIORITY_15				15
#define NVIC_IRQ_PRIORITY_16				16
#define NVIC_IRQ_PRIORITY_17				17
#define NVIC_IRQ_PRIORITY_18				18
#define NVIC_IRQ_PRIORITY_19				19
#define NVIC_IRQ_PRIORITY_20				20

////******************************************************************************************************





//-------------------------------------------------
// Registers Structures

typedef struct
{
	__vo uint32_t EVCR;			//AFIO Event Control Register
	__vo uint32_t MAPR;			//AFIO Remap/Debug IO config Register
	__vo uint32_t EXTICR[4];	//AFIO EXTI Control Registers
	__vo uint32_t MAPR2;		//AFIO Remap/Debug IO config Register
}AFIO_Regdef;

typedef struct
{
	__vo uint32_t IMR;		//EXTI Interrupt Mask Register
	__vo uint32_t EMR;		//EXTI Event Mask Register
	__vo uint32_t RTSR;		//EXTI Rising Trigger Selection Register
	__vo uint32_t FTSR;		//EXTI Falling Trigger Selection Register
	__vo uint32_t SWIER;	//EXTI Software Interrupt Event Register
	__vo uint32_t PR;		//EXTI Pending Register
}EXTI_Regdef;

typedef struct
{
	__vo uint32_t CR[2];	//GPIO Control Register Low & High (0-7) & (8-15)
	__vo uint32_t IDR;		//GPIO Input Data Register
	__vo uint32_t ODR;		//GPIO Output Data Register
	__vo uint32_t BSRR;		//GPIO Bit Set/Reset Register
	__vo uint32_t BRR;		//GPIO Port Bit Reset Register
	__vo uint32_t LCKR;		//GPIO Port Config Lock Register
}GPIO_Regdef;

typedef struct
{
	__vo uint32_t CR1;		//I2C Control Register 1
	__vo uint32_t CR2;		//I2C Control Register 2
	__vo uint32_t OAR1;		//I2C Own Address Register 1
	__vo uint32_t OAR2;		//I2C Own Address Register 2
	__vo uint32_t DR;		//I2C Data Register
	__vo uint32_t SR1;		//I2C Status Register 1
	__vo uint32_t SR2;		//I2C Status Register 2
	__vo uint32_t CCR;		//I2C Clock Control Register
	__vo uint32_t TRISE;	//I2C TRISE Register
}I2C_Regdef;

typedef struct
{
	__vo uint32_t CR;		//RCC Control Register
	__vo uint32_t CFGR;		//RCC Clock Config Register
	__vo uint32_t CIR;		//RCC Clock Interrupt Register
	__vo uint32_t APB2RSTR;	//RCC APB2 Bus Reset Register
	__vo uint32_t APB1RSTR;	//RCC APB1 Bus Reset Register
	__vo uint32_t AHBENR;	//RCC AHB Bus Enable Register
	__vo uint32_t APB2ENR;	//RCC APB2 Bus Enable Register
	__vo uint32_t APB1ENR;	//RCC APB1 Bus Enable Register
	__vo uint32_t BDCR;		//RCC Backup Domain Control Register
	__vo uint32_t CSR;		//RCC Control/Status Register
}RCC_Regdef;

typedef struct
{
	__vo uint32_t CR1;		//SPI Control Register 1
	__vo uint32_t CR2;		//SPI Control Register 2
	__vo uint32_t SR;		//SPI Status Register
	__vo uint32_t DR;		//SPI Data Register
	__vo uint32_t CRCPR;	//SPI CRC Polynomial Register
	__vo uint32_t RXCRCR;	//SPI RX CRC Register
	__vo uint32_t TXCRCR;	//SPI TX CRC Register
	__vo uint32_t I2SCFGR;	//SPI I2S Config Register
	__vo uint32_t I2SPR;	//SPI I2S Prescalar Register
}SPI_Regdef;

typedef struct
{
	__vo uint32_t SR;		//USART Status Register
	__vo uint32_t DR;		//USART Data Register
	__vo uint32_t BRR;		//USART Baud Rate Register
	__vo uint32_t CR1;		//USART Control Register 1
	__vo uint32_t CR2;		//USART Control Register 2
	__vo uint32_t CR3;		//USART Control Register 3
	__vo uint32_t GTPR;		//USART Guard Time Prescaler Register
}USART_Regdef;

/*
typedef struct
{
	__vo uint32_t
}TIM_Regdef;
*/
//-------------------------------------------------
// Base Addresses

#define PERIPH_BASE				( (uint32_t) 0x40000000 ) //Peripheral bus base addresses

#define APB1PERIPH_BASE		 	  PERIPH_BASE  			  //APB1 Peripheral base address
#define APB2PERIPH_BASE			( PERIPH_BASE + 0x10000 ) //APB2 Peripheral base address
#define AHBPERIPH_BASE			( PERIPH_BASE + 0x20000 ) //AHB Peripheral base address


// APB1 Peripherals Base Addresses

#define TIM2_BASE				( APB1PERIPH_BASE + 0x0000 ) //Timer 2 Peripheral base address
#define TIM3_BASE				( APB1PERIPH_BASE + 0x0400 ) //Timer 3 Peripheral base address
#define TIM4_BASE				( APB1PERIPH_BASE + 0x0800 ) //Timer 4 Peripheral base address
#define TIM5_BASE				( APB1PERIPH_BASE + 0x0C00 ) //Timer 5 Peripheral base address
#define TIM6_BASE				( APB1PERIPH_BASE + 0x1000 ) //Timer 6 Peripheral base address
#define TIM7_BASE				( APB1PERIPH_BASE + 0x1400 ) //Timer 7 Peripheral base address
#define TIM12_BASE				( APB1PERIPH_BASE + 0x1800 ) //Timer 12 Peripheral base address
#define TIM13_BASE				( APB1PERIPH_BASE + 0x1C00 ) //Timer 13 Peripheral base address
#define TIM14_BASE				( APB1PERIPH_BASE + 0x2000 ) //Timer 14 Peripheral base address
#define RTC_BASE				( APB1PERIPH_BASE + 0x2800 ) //RTC Peripheral base address
#define WWDG_BASE				( APB1PERIPH_BASE + 0x2C00 ) //Window Watchdog Peripheral base address
#define IWDG_BASE				( APB1PERIPH_BASE + 0x3000 ) //Independent Watchdog base address
#define SPI2_BASE				( APB1PERIPH_BASE + 0x3800 ) //SPI 2 Peripheral base address
#define SPI3_BASE				( APB1PERIPH_BASE + 0x3C00 ) //SPI 3 Peripheral base address
#define USART2_BASE				( APB1PERIPH_BASE + 0x4400 ) //USART 2 Peripheral base address
#define USART3_BASE				( APB1PERIPH_BASE + 0x4800 ) //USART 3 Peripheral base address
#define UART4_BASE				( APB1PERIPH_BASE + 0x4C00 ) //UART 4 Peripheral base address
#define UART5_BASE				( APB1PERIPH_BASE + 0x5000 ) //UART 5 Peripheral base address
#define I2C1_BASE				( APB1PERIPH_BASE + 0x5400 ) //I2C 1 Peripheral base address
#define I2C2_BASE				( APB1PERIPH_BASE + 0x5800 ) //I2C 2 Peripheral base address
#define USB_FS_BASE				( APB1PERIPH_BASE + 0x5C00 ) //USB Device FS Peripheral base address
#define CAN2_BASE				( APB1PERIPH_BASE + 0x6800 ) //CAN 2 Peripheral base address
#define CAN1_BASE				( APB1PERIPH_BASE + 0x6400 ) //I2C 2 Peripheral base address
#define BKPR_BASE				( APB1PERIPH_BASE + 0x6C00 ) //Backup Registers Peripheral base address
#define PWR_BASE				( APB1PERIPH_BASE + 0x7000 ) //POWER Peripheral base address
#define DAC_BASE				( APB1PERIPH_BASE + 0x7400 ) //DAC Peripheral base address
#define CEC_BASE              	( APB1PERIPH_BASE + 0x7800 )


// APB2 Peripherals Base Addresses

#define AFIO_BASE				( APB2PERIPH_BASE + 0x0000 ) //SYSCFG Peripheral base address
#define EXTI_BASE				( APB2PERIPH_BASE + 0x0400 ) //EXTI Peripheral base address
#define GPIOA_BASE				( APB2PERIPH_BASE + 0x0800 ) //GPIOA Peripheral base address
#define GPIOB_BASE				( APB2PERIPH_BASE + 0x0C00 ) //GPIOB Peripheral base address
#define GPIOC_BASE				( APB2PERIPH_BASE + 0x1000 ) //GPIOC Peripheral base address
#define GPIOD_BASE				( APB2PERIPH_BASE + 0x1400 ) //GPIOD Peripheral base address
#define GPIOE_BASE				( APB2PERIPH_BASE + 0x1800 ) //GPIOE Peripheral base address
#define GPIOF_BASE				( APB2PERIPH_BASE + 0x1C00 ) //GPIOF Peripheral base address
#define GPIOG_BASE				( APB2PERIPH_BASE + 0x2000 ) //GPIOG Peripheral base address
#define ADC1_BASE				( APB2PERIPH_BASE + 0x2400 ) //ADC1 Peripheral base address
#define ADC2_BASE				( APB2PERIPH_BASE + 0x2800 ) //ADC2 Peripheral base address
#define TIM1_BASE				( APB2PERIPH_BASE + 0x2C00 ) //Timer 1 Peripheral base address
#define SPI1_BASE				( APB2PERIPH_BASE + 0x3000 ) //SPI 1 Peripheral base address
#define TIM8_BASE				( APB2PERIPH_BASE + 0x3400 ) //Timer 8 Peripheral base address
#define USART1_BASE				( APB2PERIPH_BASE + 0x3800 ) //USART 1 Peripheral base address
#define ADC3_BASE				( APB2PERIPH_BASE + 0x3C00 ) //ADC3 Peripheral base address
#define TIM9_BASE				( APB2PERIPH_BASE + 0x4C00 ) //Timer 9 Peripheral base address
#define TIM10_BASE				( APB2PERIPH_BASE + 0x5000 ) //Timer 10 Peripheral base address
#define TIM11_BASE				( APB2PERIPH_BASE + 0x5400 ) //Timer 11 Peripheral base address

#define SDIO_BASE				( PERIPH_BASE + 0x18000 )

// AHB Peripherals Base Addresses

#define DMA1_BASE				( AHBPERIPH_BASE + 0x0000 )
#define DMA2_BASE				( AHBPERIPH_BASE + 0x0400 )
#define RCC_BASE				( AHBPERIPH_BASE + 0x1000 )
#define FLASH_INTERFACE_BASE	( AHBPERIPH_BASE + 0x2000 )
#define CRC_BASE				( AHBPERIPH_BASE + 0x3000 )
#define ETHERNET_BASE			( AHBPERIPH_BASE + 0x8000 )
#define USB_OTG_FS				( 0x50000000U )
#define FSMC_BASE				( 0xA0000000U )

//-------------------------------------------------
// Bit Position Macros

//GPIO_CR BITPOS
#define GPIO_CRL_MODE0					(3 << 0)
#define GPIO_CRL_MODE0_0				(1 << 0)
#define GPIO_CRL_MODE0_1				(2 << 0)
#define GPIO_CRL_MODE1					(3 << 4)
#define GPIO_CRL_MODE1_0				(1 << 4)
#define GPIO_CRL_MODE1_1				(2 << 4)
#define GPIO_CRL_MODE2					(3 << 8)
#define GPIO_CRL_MODE2_0				(1 << 8)
#define GPIO_CRL_MODE2_1				(2 << 8)
#define GPIO_CRL_MODE3					(3 << 12)
#define GPIO_CRL_MODE3_0				(1 << 12)
#define GPIO_CRL_MODE3_1				(2 << 12)
#define GPIO_CRL_MODE4					(3 << 16)
#define GPIO_CRL_MODE4_0				(1 << 16)
#define GPIO_CRL_MODE4_1				(2 << 16)
#define GPIO_CRL_MODE5					(3 << 20)
#define GPIO_CRL_MODE5_0				(1 << 20)
#define GPIO_CRL_MODE5_1				(2 << 20)
#define GPIO_CRL_MODE6					(3 << 24)
#define GPIO_CRL_MODE6_0				(1 << 24)
#define GPIO_CRL_MODE6_1				(2 << 24)
#define GPIO_CRL_MODE7					(3 << 28)
#define GPIO_CRL_MODE7_0				(1 << 28)
#define GPIO_CRL_MODE7_1				(2 << 28)

#define GPIO_CRL_CNF0					(3 << 2)
#define GPIO_CRL_CNF0_0					(1 << 2)
#define GPIO_CRL_CNF0_1					(2 << 2)
#define GPIO_CRL_CNF1					(3 << 6)
#define GPIO_CRL_CNF1_0					(1 << 6)
#define GPIO_CRL_CNF1_1					(2 << 6)
#define GPIO_CRL_CNF2					(3 << 10)
#define GPIO_CRL_CNF2_0					(1 << 10)
#define GPIO_CRL_CNF2_1					(2 << 10)
#define GPIO_CRL_CNF3					(3 << 14)
#define GPIO_CRL_CNF3_0					(1 << 14)
#define GPIO_CRL_CNF3_1					(2 << 14)
#define GPIO_CRL_CNF4					(3 << 18)
#define GPIO_CRL_CNF4_0					(1 << 18)
#define GPIO_CRL_CNF4_1					(2 << 18)
#define GPIO_CRL_CNF5					(3 << 22)
#define GPIO_CRL_CNF5_0					(1 << 22)
#define GPIO_CRL_CNF5_1					(2 << 22)
#define GPIO_CRL_CNF6					(3 << 26)
#define GPIO_CRL_CNF6_0					(1 << 26)
#define GPIO_CRL_CNF6_1					(2 << 26)
#define GPIO_CRL_CNF7					(3 << 30)
#define GPIO_CRL_CNF7_0					(1 << 30)
#define GPIO_CRL_CNF7_1					(2 << 30)

//------------------------

#define GPIO_CRH_MODE8					(3 << 0)
#define GPIO_CRH_MODE8_0				(1 << 0)
#define GPIO_CRH_MODE8_1				(2 << 0)
#define GPIO_CRH_MODE9					(3 << 4)
#define GPIO_CRH_MODE9_0				(1 << 4)
#define GPIO_CRH_MODE9_1				(2 << 4)
#define GPIO_CRH_MODE10					(3 << 8)
#define GPIO_CRH_MODE10_0				(1 << 8)
#define GPIO_CRH_MODE10_1				(2 << 8)
#define GPIO_CRH_MODE11					(3 << 12)
#define GPIO_CRH_MODE11_0				(1 << 12)
#define GPIO_CRH_MODE11_1				(2 << 12)
#define GPIO_CRH_MODE12					(3 << 16)
#define GPIO_CRH_MODE12_0				(1 << 16)
#define GPIO_CRH_MODE12_1				(2 << 16)
#define GPIO_CRH_MODE13					(3 << 20)
#define GPIO_CRH_MODE13_0				(1 << 20)
#define GPIO_CRH_MODE13_1				(2 << 20)
#define GPIO_CRH_MODE14					(3 << 24)
#define GPIO_CRH_MODE14_0				(1 << 24)
#define GPIO_CRH_MODE14_1				(2 << 24)
#define GPIO_CRH_MODE15					(3 << 28)
#define GPIO_CRH_MODE15_0				(1 << 28)
#define GPIO_CRH_MODE15_1				(2 << 28)

#define GPIO_CRH_CNF8					(3 << 2)
#define GPIO_CRH_CNF8_0					(1 << 2)
#define GPIO_CRH_CNF8_1					(2 << 2)
#define GPIO_CRH_CNF9					(3 << 6)
#define GPIO_CRH_CNF9_0					(1 << 6)
#define GPIO_CRH_CNF9_1					(2 << 6)
#define GPIO_CRH_CNF10					(3 << 10)
#define GPIO_CRH_CNF10_0				(1 << 10)
#define GPIO_CRH_CNF10_1				(2 << 10)
#define GPIO_CRH_CNF11					(3 << 14)
#define GPIO_CRH_CNF11_0				(1 << 14)
#define GPIO_CRH_CNF11_1				(2 << 14)
#define GPIO_CRH_CNF12					(3 << 18)
#define GPIO_CRH_CNF12_0				(1 << 18)
#define GPIO_CRH_CNF12_1				(2 << 18)
#define GPIO_CRH_CNF13					(3 << 22)
#define GPIO_CRH_CNF13_0				(1 << 22)
#define GPIO_CRH_CNF13_1				(2 << 22)
#define GPIO_CRH_CNF14					(3 << 26)
#define GPIO_CRH_CNF14_0				(1 << 26)
#define GPIO_CRH_CNF14_1				(2 << 26)
#define GPIO_CRH_CNF15					(3 << 30)
#define GPIO_CRH_CNF15_0				(1 << 30)
#define GPIO_CRH_CNF15_1				(2 << 30)

//RCC_AHBENR BITPOS
#define RCC_AHBENR_DMA1EN				0
#define RCC_AHBENR_DMA2EN				1
#define RCC_AHBENR_SRAMEN				2
#define RCC_AHBENR_FLITFEN				4
#define RCC_AHBENR_CRCEN				6
#define RCC_AHBENR_OTGFSEN				12
#define RCC_AHBENR_ETHMACEN				14
#define RCC_AHBENR_ETHMACTXEN			15
#define RCC_AHBENR_ETHMACRXEN			16

//RCC_APB1ENR BITPOS
#define RCC_APB1ENR_TIM2EN				0
#define RCC_APB1ENR_TIM3EN				1
#define RCC_APB1ENR_TIM4EN				2
#define RCC_APB1ENR_TIM5EN				3
#define RCC_APB1ENR_TIM6EN				4
#define RCC_APB1ENR_TIM7EN				5
#define RCC_APB1ENR_TIM12EN				6
#define RCC_APB1ENR_TIM13EN				7
#define RCC_APB1ENR_TIM14EN				8
#define RCC_APB1ENR_WWDGEN				11
#define	RCC_APB1ENR_SPI2EN				14
#define RCC_APB1ENR_SPI3EN				15
#define RCC_APB1ENR_USART2EN			17
#define RCC_APB1ENR_USART3EN			18
#define RCC_APB1ENR_UART4EN				19
#define RCC_APB1ENR_UART5EN				20
#define RCC_APB1ENR_I2C1EN				21
#define RCC_APB1ENR_I2C2EN				22
#define RCC_APB1ENR_USBEN				23
#define RCC_APB1ENR_CANEN				25
#define RCC_APB1ENR_BKPEN				27
#define RCC_APB1ENR_PWREN				28
#define RCC_APB1ENR_DACEN				29

//RCC_APB2ENR BITPOS
#define RCC_APB2ENR_AFIOEN				0
#define RCC_APB2ENR_IOAEN				2
#define RCC_APB2ENR_IOBEN				3
#define RCC_APB2ENR_IOCEN				4
#define RCC_APB2ENR_IODEN				5
#define RCC_APB2ENR_IOEEN				6
#define RCC_APB2ENR_IOFEN				7
#define RCC_APB2ENR_IOGEN				8
#define RCC_APB2ENR_ADC1EN				9
#define RCC_APB2ENR_ADC2EN				10
#define	RCC_APB2ENR_TIM1EN				11
#define RCC_APB2ENR_SPI1EN				12
#define RCC_APB2ENR_TIM8EN				13
#define RCC_APB2ENR_USART1EN			14
#define RCC_APB2ENR_ADC3EN				15
#define RCC_APB2ENR_TIM9EN				19
#define RCC_APB2ENR_TIM10EN				20
#define RCC_APB2ENR_TIM11EN				21

//SPI_CR1 BITPOS
#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL					1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSBFIRST				7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM						9
#define SPI_CR1_RXONLY					10
#define SPI_CR1_DFF						11
#define SPI_CR1_CRCNEXT					12
#define SPI_CR1_CRCEN					13
#define SPI_CR1_BIDIOE					14
#define SPI_CR1_BIDIMODE				15

//SPI_CR2 BITPOS
#define SPI_CR2_RXDMAEN					0
#define SPI_CR2_TXDMAEN					1
#define SPI_CR2_SSOE					2
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE					6
#define SPI_CR2_TXEIE					7

//SPI_SR BITPOS
#define SPI_SR_RXNE						0
#define SPI_SR_TXE						1
#define SPI_SR_CHSIDE					2
#define SPI_SR_UDR						3
#define SPI_SR_CRCERR					4
#define SPI_SR_MODF						5
#define SPI_SR_OVR						6
#define SPI_SR_BSY						7

//-------------------------------------------------
// Peripheral Registers Definition

#define AFIO					( (AFIO_Regdef *) AFIO_BASE )
#define EXTI					( (EXTI_Regdef *) EXTI_BASE )
#define GPIOA					( (GPIO_Regdef *) GPIOA_BASE )
#define GPIOB					( (GPIO_Regdef *) GPIOB_BASE )
#define GPIOC					( (GPIO_Regdef *) GPIOC_BASE )
#define GPIOD					( (GPIO_Regdef *) GPIOD_BASE )
#define GPIOE					( (GPIO_Regdef *) GPIOE_BASE )
#define GPIOF					( (GPIO_Regdef *) GPIOF_BASE )
#define GPIOG					( (GPIO_Regdef *) GPIOG_BASE )
#define I2C1					( (I2C_Regdef *) I2C1_BASE )
#define I2C2					( (I2C_Regdef *) I2C2_BASE )
#define RCC						( (RCC_Regdef *) RCC_BASE )
#define SPI1					( (SPI_Regdef *) SPI1_BASE )
#define SPI2					( (SPI_Regdef *) SPI2_BASE )
#define SPI3					( (SPI_Regdef *) SPI3_BASE )
#define USART1					( (USART_Regdef *) USART1_BASE )
#define USART2					( (USART_Regdef *) USART2_BASE )
#define USART3					( (USART_Regdef *) USART3_BASE )

//-------------------------------------------------
// Clock Function Macros

// Clock Enable
#define RCC_AFIO_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_AFIOEN )
#define RCC_GPIOA_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_IOAEN )
#define RCC_GPIOB_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_IOBEN )
#define RCC_GPIOC_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_IOCEN )
#define RCC_GPIOD_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_IODEN )
#define RCC_GPIOE_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_IOEEN )
#define RCC_GPIOF_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_IOFEN )
#define RCC_GPIOG_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_IOGEN )
#define RCC_GPIOH_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_IOHEN )

#define RCC_SPI1_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_SPI1EN )
#define RCC_SPI2_CLKEN()			RCC->APB1ENR |= ( 1 << RCC_APB1ENR_SPI2EN )
#define RCC_SPI3_CLKEN()			RCC->APB1ENR |= ( 1 << RCC_APB1ENR_SPI3EN )

#define RCC_USART1_CLKEN()			RCC->APB2ENR |= ( 1 << RCC_APB2ENR_USART1EN )
#define RCC_USART2_CLKEN()			RCC->APB1ENR |= ( 1 << RCC_APB1ENR_USART2EN )
#define RCC_USART3_CLKEN()			RCC->APB1ENR |= ( 1 << RCC_APB1ENR_USART3EN )
#define RCC_UART4_CLKEN()			RCC->APB1ENR |= ( 1 << RCC_APB1ENR_UART4EN )
#define RCC_UART5_CLKEN()			RCC->APB1ENR |= ( 1 << RCC_APB1ENR_UART5EN )

#define RCC_I2C1_CLKEN()			RCC->APB1ENR |= ( 1 << RCC_APB1ENR_I2C1EN )
#define RCC_I2C2_CLKEN()			RCC->APB1ENR |= ( 1 << RCC_APB1ENR_I2C2EN )

// Clock Disable
#define RCC_GPIOA_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_IOAEN )
#define RCC_GPIOB_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_IOBEN )
#define RCC_GPIOC_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_IOCEN )
#define RCC_GPIOD_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_IODEN )
#define RCC_GPIOE_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_IOEEN )
#define RCC_GPIOF_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_IOFEN )
#define RCC_GPIOG_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_IOGEN )
#define RCC_GPIOH_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_IOHEN )

#define RCC_SPI1_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_SPI1EN )
#define RCC_SPI2_CLKDIS()			RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_SPI2EN )
#define RCC_SPI3_CLKDIS()			RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_SPI3EN )

#define RCC_USART1_CLKDIS()			RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_USART1EN )
#define RCC_USART2_CLKDIS()			RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_USART2EN )
#define RCC_USART3_CLKDIS()			RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_USART3EN )
#define RCC_UART4_CLKDIS()			RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_UART4EN )
#define RCC_UART5_CLKDIS()			RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_UART5EN )

#define RCC_I2C1_CLKDIS()			RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_I2C1EN )
#define RCC_I2C2_CLKDIS()			RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_I2C2EN )

#define GPIO_BASE_TO_CODE(x)		( (x == GPIOA) ? 0 :\
									  (x == GPIOB) ? 1 :\
									  (x == GPIOC) ? 2 :\
									  (x == GPIOD) ? 3 :\
									  (x == GPIOE) ? 4 :\
									  (x == GPIOF) ? 5 :\
									  (x == GPIOG) ? 6 : 0 )

#define HIGH			1
#define LOW				0
#define SET				HIGH
#define RESET			LOW
#define ENABLE			SET
#define DISABLE			RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET


//-------------------------------------------------
// Driver Files

//#include "stm32f103c8_gpio_driver.h"
//#include "stm32f103c8_spi_driver.h"

#endif /* INC_STM32F103C8_H_ */
