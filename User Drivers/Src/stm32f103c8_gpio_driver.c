/*
 * stm32f103c8_gpio_driver.c
 *
 *  Created on: 29 Jun 2020
 *      Author: Dave
 */
#include "stm32f103c8_gpio_driver.h"

#define GPIO_CR_PINPOS(x)			( (x == 8) ? 0 :\
									  (x == 9) ? 1 :\
									  (x == 10) ? 2 :\
								      (x == 11) ? 3 :\
									  (x == 12) ? 4 :\
									  (x == 13) ? 5 :\
									  (x == 14) ? 6 :\
									  (x == 15) ? 7 : x )

#define GPIO_MODE_PINPOSITION		( 4 * GPIO_CR_PINPOS(pgpioHandle->pin) )
#define GPIO_MODE_BITPOS_0			( GPIO_MODE_PINPOSITION )
#define GPIO_MODE_BITPOS_1			( GPIO_MODE_BITPOS_0 + 1 )
#define GPIO_CNF_BITPOS_0			( GPIO_MODE_BITPOS_0 + 2 )
#define GPIO_CNF_BITPOS_1			( GPIO_MODE_BITPOS_0 + 3 )

//---------------------------------------------------------

void gpio_portClk(GPIO_Regdef *pGPIOx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pGPIOx == GPIOA) RCC_GPIOA_CLKEN();
		else if (pGPIOx == GPIOB) RCC_GPIOB_CLKEN();
		else if (pGPIOx == GPIOC) RCC_GPIOC_CLKEN();
		else if (pGPIOx == GPIOD) RCC_GPIOD_CLKEN();
		else if (pGPIOx == GPIOE) RCC_GPIOE_CLKEN();
		else if (pGPIOx == GPIOF) RCC_GPIOF_CLKEN();
		else if (pGPIOx == GPIOG) RCC_GPIOG_CLKEN();
	}

	else
	{
		if (pGPIOx == GPIOA) RCC_GPIOA_CLKDIS();
		else if (pGPIOx == GPIOB) RCC_GPIOB_CLKDIS();
		else if (pGPIOx == GPIOC) RCC_GPIOC_CLKDIS();
		else if (pGPIOx == GPIOD) RCC_GPIOD_CLKDIS();
		else if (pGPIOx == GPIOE) RCC_GPIOE_CLKDIS();
		else if (pGPIOx == GPIOF) RCC_GPIOF_CLKDIS();
		else if (pGPIOx == GPIOG) RCC_GPIOG_CLKDIS();
	}
}

//---------------------------------------------------------

void gpio_init(GPIO_Handle gpioHandle)
{
	GPIO_Handle *pgpioHandle = &gpioHandle;

	gpio_portClk(pgpioHandle->port, ENABLE);	//Enable clock for GPIOx

	uint8_t CRx;
	CRx = pgpioHandle->pin / 8;

	//-----------------------------------------------------
	// NON-INTERRUPT MODE

	if (pgpioHandle->mode <= GPIO_MODE_OUTPUT)
	{
		// MODE TYPE
		switch(pgpioHandle->modeType)
		{
			case GPIO_MODETYPE_IN_ANALOG | GPIO_MODETYPE_OUT_PUPL:
				pgpioHandle->port->CR[CRx] &= ~( (1 << GPIO_CNF_BITPOS_0) | (1 << GPIO_CNF_BITPOS_1) );
				break;

			case GPIO_MODETYPE_IN_FLOATING | GPIO_MODETYPE_OUT_OD:
				pgpioHandle->port->CR[CRx] &= ~(1 << GPIO_CNF_BITPOS_1);
				pgpioHandle->port->CR[CRx] |= (1 << GPIO_CNF_BITPOS_0);
				break;

			case GPIO_MODETYPE_IN_INPUT_PUPD | GPIO_MODETYPE_OUTAF_PUPL:
				pgpioHandle->port->CR[CRx] &= ~(1 << GPIO_CNF_BITPOS_0);
				pgpioHandle->port->CR[CRx] |= (1 << GPIO_CNF_BITPOS_1);
				break;

			case GPIO_MODETYPE_OUTAF_OD:
				pgpioHandle->port->CR[CRx] |= ( GPIO_MODETYPE_OUTAF_OD << GPIO_CNF_BITPOS_0 );
				break;
		}

		// MODE & SPEED
		if(pgpioHandle->mode == GPIO_MODE_INPUT){
			pgpioHandle->port->CR[CRx] &= ~( (1 << GPIO_MODE_BITPOS_0) | (1 << GPIO_MODE_BITPOS_1) );
		}
		else{
			pgpioHandle->port->CR[CRx] |= (pgpioHandle->speed << GPIO_MODE_BITPOS_0);
		}
	}

	//-----------------------------------------------------
	// INTERRUPT MODE

	else
	{
		RCC_AFIO_CLKEN();		//Enable clock for AFIO

		// Edge Trigger Settings
		if(pgpioHandle->modeType == GPIO_MODETYPE_IT_FALLING)
		{
			//configure FTSR (Falling Trigger Selection Register)
			EXTI->RTSR &= ~( 1 << pgpioHandle->pin ); //disable Rising Trigger
			EXTI->FTSR |= ( 1 << pgpioHandle->pin ); //enable Falling Trigger
		}

		else if(pgpioHandle->modeType == GPIO_MODETYPE_IT_RISING)
		{
			//configure RTSR (Rising Trigger Selection Register)
			EXTI->FTSR &= ~( 1 << pgpioHandle->pin ); //disable Falling Trigger
			EXTI->FTSR |= ( 1 << pgpioHandle->pin ); //enable Rising Trigger
		}

		else if(pgpioHandle->modeType == GPIO_MODETYPE_IT_RISING_FALLING)
		{
			//configure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << pgpioHandle->pin ); //enable Falling Trigger
			EXTI->FTSR |= ( 1 << pgpioHandle->pin ); //enable Rising Trigger
		}

		//configure which GPIO port selection in AFIO_EXTICR[]
		uint8_t temp1 = pgpioHandle->pin / 4;
		uint8_t temp2 = pgpioHandle->pin % 4;
		AFIO->EXTICR[temp1] |= GPIO_BASE_TO_CODE(pgpioHandle->port) << ( 4 * temp2 );

		//enable EXTI interrupt delivery with IMR (Interrupt Mask Register)
		EXTI->IMR |= ( 1 << pgpioHandle->pin );

	}
}


//---------------------------------------------------------

uint8_t gpio_read(GPIO_Handle gpioHandle)
{
	GPIO_Handle *pgpioHandle = &gpioHandle;
	uint8_t value;
	value = (uint8_t)(pgpioHandle->port->IDR >> pgpioHandle->pin) & 0x00000001;
	return value;
}

//---------------------------------------------------------

uint16_t gpio_readPort(GPIO_Regdef *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}

//---------------------------------------------------------

void gpio_write(GPIO_Handle gpioHandle, uint8_t state)
{
	GPIO_Handle *pgpioHandle = &gpioHandle;
	if(state == HIGH) pgpioHandle->port->ODR |= (1 << pgpioHandle->pin);
	else pgpioHandle->port->ODR &= ~(1 << pgpioHandle->pin);
}

//---------------------------------------------------------

void gpio_writePort(GPIO_Regdef *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

//---------------------------------------------------------

void gpio_toggle(GPIO_Handle gpioHandle)
{
	GPIO_Handle *pgpioHandle = &gpioHandle;
	pgpioHandle->port->ODR ^= (1 << pgpioHandle->pin);
}

/*---------------------------------------------------------
				INTERRUPT DRIVERS
---------------------------------------------------------*/

void gpio_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	if(IRQNumber > NVIC_IRQ_PRIORITY_DEFAULT)
	{
		//find out the IPR register
		uint8_t IPRx = IRQNumber / 4;
		uint8_t IPRx_section = IRQNumber % 4;

		uint8_t shiftAmount = ( 8 * IPRx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
		*( CPU_MVIC_IPR_BASE + IPRx ) |= ( IRQPriority << shiftAmount  );
	}

	else;
}

void gpio_IRQInterrupt(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t enOrDis)
{
	gpio_IRQPriorityConfig(IRQNumber, IRQPriority);

	if(enOrDis == ENABLE)
	{
		if(IRQNumber <= 31) *CPU_NVIC_ISER0 |= ( 1 << IRQNumber ); //IRQNumber =  0-31, program ISER0 register to enable IRQ number
		else if(IRQNumber > 31 && IRQNumber < 64) *CPU_NVIC_ISER1 |= ( 1 << (IRQNumber % 32) ); //IRQNumber =  32-63, program ISER1 register to enable IRQ number
		else if(IRQNumber >= 64 && IRQNumber < 96) *CPU_NVIC_ISER2 |= ( 1 << (IRQNumber % 64) ); //IRQNumber =  64-95, program ISER2 register to enable IRQ number
	}

	else
	{
		if(IRQNumber <= 31) *CPU_NVIC_ICER0 |= ( 1 << IRQNumber ); //IRQNumber =  0-31, program ICER0 register
		else if(IRQNumber > 31 && IRQNumber < 64) *CPU_NVIC_ICER1 |= ( 1 << (IRQNumber % 32) ); //IRQNumber =  32-63, program ICER1 register to disable IRQ number
		else if(IRQNumber >= 64 && IRQNumber < 96) *CPU_NVIC_ICER2 |= ( 1 << (IRQNumber % 64) ); //IRQNumber =  64-95, program ICER2 register to disable IRQ number
	}
}

void gpio_IRQhandling(uint8_t pinNumber)
{
	//clear EXTI Pending Register corresponding to the pin
	if(EXTI->PR & ( 1 << pinNumber ) ) EXTI->PR |= ( 1 << pinNumber );
}



