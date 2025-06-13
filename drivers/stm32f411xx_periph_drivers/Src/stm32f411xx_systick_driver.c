/*
 * stm32f411xx_systick_driver.c
 *
 *  Created on: Feb 23, 2025
 *      Author: grudz
 *
 *      TODO	add function descriptions
 */
#include "stm32f411xx.h"
#include "stm32f411xx_systick_driver.h"

/* For RTOS functions */
#include "../drivers/Simple_RTOS/simplertos.h"

/*
 * global variable for tick count
 */
static uint32_t volatile l_tickCtr;


/***************************************************************
 * 				FUNCTION DESCRIPTION
 * @fn				- SysTick_Init
 *
 * @brief			- Initialize SysTick - set default values to registers
 *
 * @param EnOrDis	- ENABLE / DISABLE macro
 *
 * @return			-
 *
 * @Note			-
 *
 ***************************************************************/
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
 * 					FUNCTION DESCRIPTION
 * @fn				- check_SysTick
 *
 * @brief			-
 *
 * @param			- none
 *
 * @return tickCtr	-
 *
 * @Note			- none
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
 * 					FUNCTION DESCRIPTION
 * @fn			- SysTick_Handler
 *
 * @brief		-
 *
 * @param		- none
 *
 * @return		- none
 *
 * @Note		- If using RTOS uncomment RTOS functions
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
 * 					FUNCTION DESCRIPTION
 * @fn				-
 *
 * @brief			-
 *
 * @param ticks_ms	-
 *
 * @return			-
 *
 * @Note			-
 *
 ******************************************************************/
void simple_delay(uint32_t ticks_ms) {
	uint32_t start = check_SysTick();
	while((check_SysTick() - start) < ticks_ms)	{}
}




/******************************************************************
 * 					FUNCTION DESCRIPTION
 * @fn					- Systick_Read_Countflag
 *
 * @brief				- Bit 16 of STK_CTRL returns 1 if timer
 * 						  counted to 0 since the last time it was read.
 *
 * @param				- none
 *
 * @return CntFlag_val	-
 *
 * @Note				-
 *
 ******************************************************************/
uint8_t Systick_Read_Countflag(void)
{
		uint8_t CntFlag_val;
		CntFlag_val = (uint8_t) ((SysTick->STK_CTRL >> 16) & 0x00000001);

		return CntFlag_val;
}

