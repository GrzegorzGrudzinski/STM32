/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Jul 26, 2024
 *      Author: grudz
 *
 *      TODO add function descriptions and comments
 */

#include "stm32f411xx_gpio_driver.h"

//Peripheral Clock setup

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_PeriClockControl
 *
 * @brief					-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in] pGPIOx		-	base address of the gpio peripheral
 * @param[in] EnORDis		-	ENABLE / DISABLE macros
 *
 * @return					-	none
 *
 * @Note					-	none
 *
 ******************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnORDis)
{
	if(EnORDis == ENABLE)
	{
		//enable the pin
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
	}
	else
	{
		//disable the pin
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
	}
}

//Init and De-init

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_Init
 *
 * @brief					- this funct. initializes the gpio - sets up its speed, mode etc.
 *
 * @param[in] pGPIOHandle	- gpio_handle struct - basic settings of a gpio
 *
 * @return					- none
 *
 * @Note					- none
 *
 ******************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)	//non-interrupt mode
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// (2*...) - 2 bit fields
		//clearing the bitfield (negate the value of 1 1)
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//2
		//setting
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure the FTSR and FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;	//select the exti register (EXTICR)
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;	//select the exti number (EXTIx)
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (portcode << (temp2 * 4));

		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. configure the pull up / down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. configure the alt. functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN)
	{
		//configure the alt. funct. registers
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;	//afrl or afrh (0 / 1)
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode	% 4;	//for storing shift number (pin number)
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_DeInit
 *
 * @brief					-	function that resets the gpio registers
 *
 * @param[in] pGPIOx		-	base addr of the gpio
 *
 * @return					-	none
 *
 * @Note					-	none
 *
 ******************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
}

//Data read and write

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_ReadFromInputPin
 *
 * @brief					-  read the value a GPIO pin
 *
 * @param[in] pGPIOx		- base addr of the gpio (gpio port)
 * @param[in] PinNumber		- pin number
 *
 * @return					- pin value
 *
 * @Note					- none
 *
 ******************************************************************/
uint8_t	GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
		uint8_t value;
		value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);

		return value;
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_ReadFromInputPort
 *
 * @brief					-  read the input data register of a GPIO port
 *
 * @param[in] pGPIOx		-  base addr of the gpio (gpio port)
 *
 * @return					-  gpio port value (IDR reg. content)
 *
 * @Note					- none
 *
 ******************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)	//16 pins
{
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);

	return value;
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_WriteToOutputPin
 *
 * @brief					-
 *
 * @param[in] pGPIOx		-
 * @param[in] PinNumber		-
 * @param[in] Value			-
 *
 * @return					-
 *
 * @Note					-
 *
 ******************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
		pGPIOx->ODR |= (1 << PinNumber);	//write 1 to the output data reg. at the bit field corresponding to the pin number
	else
		pGPIOx->ODR &= ~(1 << PinNumber);	//clear the bit position
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_WriteToOutputPort
 *
 * @brief					-
 *
 * @param[in] pGPIOx		-
 * @param[in] Value			-
 *
 * @return					-
 *
 * @Note					-
 *
 ******************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_ToggleOutputPin
 *
 * @brief					-
 *
 * @param[in] pGPIOx		-
 * @param[in] PinNumber		-
 *
 * @return					-
 *
 * @Note					-
 *
 ******************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

//IRQ Configuration and ISR handling

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_IRQInterruptConfig
 *
 * @brief					-
 *
 * @param[in] IRQNumber		-
 * @param[in] EnORDis		-
 *
 * @return					-
 *
 * @Note					-
 *
 ******************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnORDis)
{
	if(EnORDis == ENABLE)
	{ //Enable IRQ
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)	//32 to 64
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );

		}
	}//if
	else
	{ //Disable IRQ
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 32) );
		}
	}
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_IRQPriorityConfig
 *
 * @brief					-
 *
 * @param[in] IRQNumber		-
 * @param[in] IRQPriority	-
 *
 * @return					-
 *
 * @Note					-
 *	TODO	Move to nvic lib ??
 ******************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	//only 4 bits are implemented, so there is need to shift by 4
	*(NVIC_PR_BASE_ADDR + (iprx)) |= ( IRQPriority << shift_amount );
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_IRQHandling
 *
 * @brief					-	clears the exti pr register corresponding to the pin number
 *
 * @param[in] PinNumber		-	Pin number
 *
 * @return					-	none
 *
 * @Note					-	This function should be called from Interrupt Handler
 *
 ******************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber )) //if the PR bit position corresponding to the pin number is set
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}

