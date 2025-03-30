/*
 * stm32f411xx_nvic_driver.c
 *
 *  Created on: Mar 10, 2025
 *      Author: grudz
 *
 *      TODO	add function descriptions
 *      TODO	Finish nvic library
 */
#include "stm32f411xx.h"



/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 * @param[in]				-
 *
 * @return					-
 *
 * @Note					-
 *
 ******************************************************************/
void NVIC_EnableIRQ(uint8_t IRQNumber)
{

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
}


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	GPIO_IRQHandling
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					-
 * b
 * @Note					-
 *
 ******************************************************************/
void NVIC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	//only 4 bits are implemented, so there is need to shift by 4
	*(NVIC_PR_BASE_ADDR + (iprx)) |= ( IRQPriority << shift_amount );
}
