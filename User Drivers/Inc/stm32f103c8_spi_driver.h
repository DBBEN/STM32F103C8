/*
 * stm32f103c8_spi_driver.h
 *
 *  Created on: 1 Jul 2020
 *      Author: Dave
 */

#ifndef INC_STM32F103C8_SPI_DRIVER_H_
#define INC_STM32F103C8_SPI_DRIVER_H_

#include "stm32f103c8.h"

//SPI Device Mode Macros
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

//SPI Bus Config Macro
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SMPLX_RXONLY		3

//SPI SCLK Speed Macro
#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

//SPI DFF Macro (Data Frame Format / Data Size)
#define SPI_DFF_4BITS							3
#define SPI_DFF_5BITS							4
#define SPI_DFF_6BITS							5
#define SPI_DFF_7BITS							6
#define SPI_DFF_8BITS							7
#define SPI_DFF_9BITS							8
#define SPI_DFF_10BITS							9
#define SPI_DFF_11BITS							10
#define SPI_DFF_12BITS							11
#define SPI_DFF_13BITS							12
#define SPI_DFF_14BITS							13
#define SPI_DFF_15BITS							14
#define SPI_DFF_16BITS							15

//SPI CPOL	(Clock Polarity) on IDLE
#define SPI_CPOL_HIGH							1
#define SPI_CPOL_LOW							0

//SPI CPHA	(Clock Phase)
#define SPI_CPHA_HIGH							1
#define SPI_CPHA_LOW							0

//SPI SSM	(Slave Select Management)
#define SPI_SSM_EN								1
#define SPI_SSM_DIS								0



//Masking details for the Status Register
#define SPI_TXE_FLAG							( 1 << 1 )
#define SPI_RXE_FLAG							( 1 << 0 )
#define SPI_BUSY_FLAG							( 1 << 7 )

//	Handle Structure for SPIx Peripheral
typedef struct
{
	SPI_Regdef	*pSPIx;
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Handle;

/*******************************************************************
 * 				API FUNCTION PROTOTYPES
 *******************************************************************/

//Peripheral Clock Setup
void spi_portClk(SPI_Regdef SPIx, uint8_t enOrDis);

//Init and De-init
void spi_init(SPI_Handle SPIHandle);
void spi_deInit(SPI_Regdef SPIx);
void spi_SSIConfig(SPI_Regdef SPIx, uint8_t enOrDis);
void spi_SSOEConfig(SPI_Regdef SPIx, uint8_t enOrDis);
void spi_NSSPConfig(SPI_Regdef SPIx, uint8_t enOrDis);

//Data Send and Receive
void spi_beginCommunication(SPI_Regdef SPIxpSPIx);
void spi_endCommunication(SPI_Regdef SPIx);
void spi_sendData(SPI_Regdef SPIx, uint8_t *pTXbuffer, uint32_t Len);
void spi_receiveData(SPI_Regdef SPIx, uint8_t *pRXbuffer, uint32_t Len);
uint8_t spi_getFlagStatus(SPI_Regdef SPIx, uint32_t FlagName);


//IRQ Configuration and ISR handling
void spi_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void spi_IRQInterruptConfig(uint8_t IRQNumber,uint8_t enOrDis);
void spi_IRQhandling(SPI_Handle *pHandle);

#endif /* INC_STM32F103C8_SPI_DRIVER_H_ */
