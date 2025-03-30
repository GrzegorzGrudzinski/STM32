/*
 * stm32f411xx_systick_driver.c
 *
 *  Created on: Feb 23, 2025
 *      Author: grudz
 *
 *      TODO	add function descriptions
 *      TODO    repair rtos lib inc
 */
#include "stm32f411xx.h"
#include "stm32f411xx_systick_driver.h"

/* For RTOS functions */
#include "../Simple_RTOS/simplertos.h"

/*******************************************************
 *
 *  					VARIABLES
 *
 ******************************************************/

/* global variable for tick count  */
static uint32_t volatile l_tickCtr;

/*******************************************************
 *
 *  					FUNCTIONS
 *
 ******************************************************/


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-
 *
 * @brief					- Initialize SysTick - set default values to registers
 *
 * @param[in] EnOrDis       -
 *
 * @return					- NONE
 *
 * @Note					-
 *
 ******************************************************************/
void SysTick_Init (uint8_t EnOrDis)
{
	//Bit 0 - Enable/Disable systick, Bit 1 TICKINT: SysTick exception request enable, Bit 2 CLKSOURCE: Clock source selection
	SysTick->STK_CTRL = (EnOrDis << 0) | (1u << 1) | (1u << 2);
	SysTick->STK_LOAD = SYS_CLK_HZ / TICKS_PER_SEC;
	SysTick->STK_VAL  = 0u;
	//	SysTick->STK_CALIB = 0u;

	/*	set the SysTick interrupt priority to the higher level	*/
	*(uint32_t volatile *)0xE000ED20 &= ~(0xFFU << 24);	//clear the bits
	*(uint32_t volatile *)0xE000ED20 |= (0x00U << 24);	//set priority to 0
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-
 *
 * @brief					-
 *
 * @param[in]				-
 *
 * @return					-
 *
 * @Note					-
 *
 ******************************************************************/
uint32_t check_SysTick(void) {
	uint32_t tickCtr;

	SysTick->STK_CTRL |= (0U << 0);
	tickCtr = l_tickCtr;
	SysTick->STK_CTRL |= (1U << 0);

	return tickCtr;
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-
 *
 * @brief					-
 *
 * @param[in]				-
 *
 * @return					-
 *
 * @Note					- If using RTOS uncomment RTOS functions
 *
 ******************************************************************/
void SysTick_Handler(void) {
	++l_tickCtr;

	/* RTOS functions  */
	disable_irq();
	OS_sched();
	enable_irq();
}

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-
 *
 * @brief					-
 *
 * @param[in] ticks			-
 *
 * @return					-
 *
 * @Note					-
 *
 ******************************************************************/
void simple_delay(uint32_t ticks) {
	uint32_t start = check_SysTick();
	while((check_SysTick() - start) < ticks)	{}
}




/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-
 *
 * @brief					- Bit 16 of STK_CTRL returns 1 if timer counted to 0 since last time this was read.
 *
 * @param[in]				-
 *
 * @return					-
 *
 * @Note					-
 *
 ******************************************************************/
uint8_t Systick_Read_Countflag(void)
{
		uint8_t value;
		value = (uint8_t) ((SysTick->STK_CTRL >> 16) & 0x00000001);

		return value;
}

