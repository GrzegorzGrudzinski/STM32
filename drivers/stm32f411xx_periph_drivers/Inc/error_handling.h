/*
 * error_handling.c
 *
 *  Created on: Mar 5, 2025
 *      Author: grudz
 */

#ifndef ERROR_HANDLING_H_
#define ERROR_HANDLING_H_

#include "stm32f411xx.h"

//SysReset					0xE000ED0C
#define SCB_BASEADDR		( 0xE000ED00U )
#define AIRCR_ADDR			(*((__vo uint32_t*)(SCB_BASEADDR + 0x0CU)))


/*
 * disable ...
 */
static inline void __DSB(void)
{
    __asm volatile ("DSB 0xF":::"memory");
}


void Sys_Reset(void);



#endif /* ERROR_HANDLING_H_ */
