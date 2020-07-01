/*
 * stm32f103c8_gpio_driver.h
 *
 *  Created on: 29 Jun 2020
 *      Author: Dave
 */

#ifndef INC_STM32F103C8_GPIO_DRIVER_H_
#define INC_STM32F103C8_GPIO_DRIVER_H_

#include "stm32f103c8.h"

typedef struct
{
	GPIO_Regdef *port;
	uint8_t pin;
	uint8_t speed;
	uint8_t mode;
	uint8_t modeType;
	uint8_t AFmode;
}GPIO_Handle;

#define GPIO_PIN0			0
#define GPIO_PIN1			1
#define GPIO_PIN2			2
#define GPIO_PIN3			3
#define GPIO_PIN4			4
#define GPIO_PIN5			5
#define GPIO_PIN6			6
#define GPIO_PIN7			7
#define GPIO_PIN8			8
#define GPIO_PIN9			9
#define GPIO_PIN10			10
#define GPIO_PIN11			11
#define GPIO_PIN12			12
#define GPIO_PIN13			13
#define GPIO_PIN14			14
#define GPIO_PIN15			15

//----------------------------------------------
//		@ GPIO_MODES

#define GPIO_MODE_INPUT							0
#define GPIO_MODE_OUTPUT						1
#define GPIO_MODE_INTERRUPT						2

//----------------------------------------------
//		@ GPIO_MODE_TYPES

// Input
#define GPIO_MODETYPE_IN_ANALOG					0
#define GPIO_MODETYPE_IN_FLOATING				1
#define GPIO_MODETYPE_IN_INPUT_PUPD				2
// Output
#define GPIO_MODETYPE_OUT_PUPL					0
#define GPIO_MODETYPE_OUT_OD					1
#define GPIO_MODETYPE_OUTAF_PUPL				2
#define GPIO_MODETYPE_OUTAF_OD					3

#define GPIO_MODETYPE_IT_RISING					4
#define GPIO_MODETYPE_IT_FALLING				5
#define GPIO_MODETYPE_IT_RISING_FALLING			6

//----------------------------------------------
//		@ GPIO_OP_SPEED

#define GPIO_OPSPEED_10MHZ			1
#define GPIO_OPSPEED_2MHZ			2
#define GPIO_OPSPEED_50MHZ			3


/*******************************************************************
 * 				API FUNCTION PROTOTYPES
 *******************************************************************/

void gpio_portClk(GPIO_Regdef *pGPIOx, uint8_t state);

void gpio_init(GPIO_Handle gpioHandle);
//void gpio_deInit(GPIO_Handle gpioHandle);

uint8_t gpio_read(GPIO_Handle gpioHandle);
uint16_t gpio_readPort(GPIO_Regdef *pGPIOx);
void gpio_write(GPIO_Handle gpioHandle, uint8_t state);
void gpio_writePort(GPIO_Regdef *pGPIOx, uint16_t value);
void gpio_toggle(GPIO_Handle gpioHandle);

void gpio_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void gpio_IRQInterrupt(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t enOrDis);
void gpio_IRQhandling(uint8_t pinNumber);

#endif /* INC_STM32F103C8_GPIO_DRIVER_H_ */
