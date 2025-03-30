/*
 * Error_handling.c
 *
 *  Created on: Mar 5, 2025
 *      Author: grudz
 *
 *      TODO add function descriptions and comments
 */

#include "error_handling.h"



/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						- Sys_Reset
 *
 * @brief					- Trigger the software reset
 *
 * @param[in]				- none
 *
 * @return					- none
 *
 * @Note					- none
 *
 ******************************************************************/
void Sys_Reset(void) {
	__DSB();
	// trigger the SYSRESETREQ (bit 2)
	AIRCR_ADDR = 0x05FA0004U;
	__DSB();
//	AIRCR_ADDR |= (1U << 2);
	while(1);
}



