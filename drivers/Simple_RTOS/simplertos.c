/*
 * simplertos.c
 *
 *  Created on: Feb 23, 2025
 *      Author: grudz
 *
 *      TODO naprawic obsluge bledow
 */

#include <stdint.h>
#include "simplertos.h"
#include "../Inc/error_handling.h"

/*
 * Max amount of possible threads to run.
 * previously: 32+1 , +1 because ...
 */
#define MAX_THREAD			32

/*	pointer to the current thread	*/
OSThread * volatile OS_curr;
/*	pointer to the next thread to run	*/
OSThread * volatile OS_next;
/*	array of started threads	*/	//RTOS can handle up to 32 threads
OSThread * OS_thread[MAX_THREAD];
/*	number of started threads */
uint8_t	OS_threadNum;
/*	current thread index	*/
uint8_t	OS_curr_threadIdx;
/* group the ready to execute threads */
uint32_t OS_readySet;

//todo - register structs
//System control block (SCB)
#define SHPR3_BASEADDR		0xE000ED20	//System handler priority registers
#define ICSR_BASEADDR		0xE000ED04	//interrupt control and state reg


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
//uint32_t stack_idleThread[40];
OSThread idleThread;
void main_idleThread() {
	while(1) {
		OS_onIdle();
	}
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
void OS_delay(uint32_t ticks) {
	disable_irq(); //disable interrupts

	/* never call OS_delay from the idleThread */
	if(OS_curr == OS_thread[0]) {
		OS_Error_Handler(1);
	}

	OS_curr->timeout = ticks; //store amount of delay time
	OS_readySet &= ~(1U << (OS_curr_threadIdx - 1U)); //set the thread as not ready
	OS_sched(); //immediately switch the context
	enable_irq();
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
void OS_init(void *stkSto, uint32_t stkSize)
{
	disable_irq();
	/*	set the PendSV interrupt priority to the lowest level	*/
	*(uint32_t volatile *)SHPR3_BASEADDR |= (0xFFU << 16);

	OS_thread_create(&idleThread,
			&main_idleThread,
			stkSto, sizeof(stkSize));

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
void OS_sched(void) {
	/*	for switching more threads you can use linked list - todo later ??	*/
	/* just for the two threads
	extern OSThread blinky1, blinky2;
	if (OS_curr == &blinky1) {
		OS_next = &blinky2;
	} else {
		OS_next = &blinky1;
	} */
	/*	storing thread pointers in pre-allocated array	*/
	/*	++OS_curr_threadIdx;
	if (OS_curr_threadIdx == OS_threadNum) 	{
		OS_curr_threadIdx = 0U;
	}*/

	if (OS_readySet == 0U) {
		/* if no threads are ready set current thread as idleThread */
		OS_curr_threadIdx = 0U;
	}
	else {
		do {
			++OS_curr_threadIdx;
			if ( OS_curr_threadIdx == OS_threadNum ) {
				//after executing all threads go back to the first thread
				OS_curr_threadIdx = 1U;
			} //if
		} while ( ( OS_readySet & ( 1U << (OS_curr_threadIdx - 1U) ) ) == 0U );
	} //else
	OS_next = OS_thread[OS_curr_threadIdx];

	if(OS_next != OS_curr)	{ //
		*(uint32_t volatile *)ICSR_BASEADDR |= (0x1U << 28); //28 - pendsvset
	}
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
void OS_run(void)	{
	/*	callback to configure and start interrupts	*/
//	OS_onStartup();

	disable_irq();
	OS_sched();
	enable_irq();

	//this code should be never executed
	OS_Error_Handler(0);
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
void OS_tick() {
	//this function is assumed to be called from an interrupt handler
	uint8_t n;
	for (n = 1U; n < OS_threadNum; ++n) {
		--OS_thread[n]->timeout;
		if (OS_thread[n]->timeout != 0U) {
			OS_readySet |= (1U << (n-1U) );
		}
	} //for
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
void OS_thread_create(
		OSThread *me,
		OSThreadHandler threadHandler,
		void *stkSto, uint32_t stkSize)
{
	/*	round down the stack top to the 8-byte boundary	(alt. (uint32_t) sp &= ~0x7;) */
	uint32_t *sp = (uint32_t *) ((((uint32_t) stkSto + stkSize) / 8) * 8);
	uint32_t *stk_limit;

	/*	allocate stack frame	*/
	*(--sp) = ( 1U << 24 ); /* xPSR */
	*(--sp) = (uint32_t)threadHandler; /* PC */
	*(--sp) = 0x0000000EU	;	/* LR */
	*(--sp) = 0x0000000CU	;	/* R12 */
	*(--sp) = 0x00000003U	;	/* R3 */
	*(--sp) = 0x00000002U	;	/* R2 */
	*(--sp) = 0x00000001U	;	/* R1 */
	*(--sp) = 0x00000000U	;	/* R0 */

	/*	allocate extra registers	*/
	*(--sp) = 0x0000000BU	;	/* R11 */
	*(--sp) = 0x0000000AU	;	/* R10 */
	*(--sp) = 0x00000009U	;	/* R9 */
	*(--sp) = 0x00000008U	;	/* R8 */
	*(--sp) = 0x00000007U	;	/* R7 */
	*(--sp) = 0x00000006U	;	/* R6 */
	*(--sp) = 0x00000005U	;	/* R5 */
	*(--sp) = 0x00000004U	;	/* R4 */

	/*	save the top of the stack in the thread's attribute	*/
	me->sp = sp;

	/*	round up the bottom of the stac to the 8-byte boundary	 */
	stk_limit = (uint32_t *) (((((uint32_t) stkSto - 1U) / 8) + 1U ) * 8);

	/*	pre-fill the unused part of the stack with 0xDEADBEEF	 */
	for (sp = sp - 1U; sp >= stk_limit; --sp) 	{
		*sp = 0xDEADBEEFU;
	}

	/*	check if the array is not overflown	*/
	if (OS_threadNum < (MAX_THREAD - 1) ) 	{
		/*	register the thread with the OS	*/
		OS_thread[OS_threadNum] = me;

		/* make thread ready to run */
		if(OS_threadNum > 0U) {
			OS_readySet |= (1U << (OS_threadNum - 1U) );
		}
		++OS_threadNum;
	}
	else {
		/*	if an array was overflown return error code and reset the system	*/
		OS_Error_Handler(0);
	}
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
 * @Note					- PendSV is an interrupt-driven request for system-level service. In an
							  OS environment, use PendSV for context switching when no other
							  exception is active
*
 ******************************************************************/
__attribute__ ((naked))
void PendSV_Handler(void) {

    __asm volatile (
        /* disable_irq(); */
        "    CPSID    I                  \n"

        /* if (OS_curr != (OSThread *)0) */
        "    LDR      r1, =OS_curr        \n"
        "    LDR      r1, [r1]            \n"
        "    CMP      r1, #0              \n"
        "    BEQ      PendSV_restore      \n"

        /* Saving context: registers r4-r11 */
        "    STMDB    sp!, {r4-r11}       \n"

        /* OS_curr->sp = sp */
        "    LDR      r1, =OS_curr        \n"
        "    LDR      r1, [r1]            \n"
        "    STR      sp, [r1]            \n"

    "PendSV_restore:                     \n"
        /* if (OS_next == 0) return; */
        "    LDR      r1, =OS_next        \n"
        "    LDR      r1, [r1]            \n"
        "    CMP      r1, #0              \n"
        "    BEQ      PendSV_exit         \n"

        /* sp = OS_next->sp */
        "    LDR      sp, [r1]            \n"

        /* OS_curr = OS_next */
        "    LDR      r2, =OS_curr        \n"
        "    STR      r1, [r2]            \n"

        /* Restoring context */
        "    LDMIA    sp!, {r4-r11}       \n"

    "PendSV_exit:                         \n"
        /* enable_irq(); */
        "    CPSIE    I                  \n"

        /* return to the next thread */
        "    BX       lr                  \n"
    );
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
void OS_Error_Handler(uint8_t Error_Code) {
	switch (Error_Code) 	{
		/*	array of threads is owerflown	*/
		case 0:
			/* ERROR CODE */
			Sys_Reset();
			break;
		/* user tried to block the idleThread */
		case 1:
			/* ERROR CODE */
			Sys_Reset();
			break;

		default:
			Sys_Reset();
			break;

	}
}

