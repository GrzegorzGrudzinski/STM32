/*
 * simplertos.h
 *
 *  Created on: Feb 23, 2025
 *      Author: grudz
 */

#ifndef SIMPLERTOS_H_
#define SIMPLERTOS_H_



static inline void disable_irq(void) {
    __asm volatile ("CPSID i" ::: "memory");
}

static inline void enable_irq(void) {
    __asm volatile ("CPSIE i" ::: "memory");
}


/* Thread Control BLock (TCB)	*/
typedef struct {
	void *sp; /* stack pointer */
	uint32_t timeout; /* down counter for the thread  */
	/*		*/
} OSThread;

/*
 *
 */
typedef void (*OSThreadHandler)();

/*  */
void OS_init(void *stkSto, uint32_t stkSize);

/*	This function must be called with interrupts DISABLED	*/
void OS_sched(void);

/* after configuring threads start the program (enable interrupts) - transfer control to rtos	*/
void OS_run(void);

void OS_onIdle(void);

void OS_thread_create(
		OSThread *me,
		OSThreadHandler threadHandler,
		void *stkSto, uint32_t stkSize);

void OS_Error_Handler(uint8_t Error_Code);

#endif /* SIMPLERTOS_H_ */
