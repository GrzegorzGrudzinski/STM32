/*
 * stm32f411xx_systick_driver.h
 *
 *  Created on: Feb 23, 2025
 *      Author: grudz
 */

#ifndef INC_STM32F411XX_SYSTICK_DRIVER_H_
#define INC_STM32F411XX_SYSTICK_DRIVER_H_

/**************************************************************************************
 *																					  *
 * 										MACROS										  *
 *																					  *
 **************************************************************************************/
/*
 *
 */
#define SYS_CLK_HZ			16000000U
#define TICKS_PER_SEC		100U

/*
 * Time conversion macros
 */
#define SECONDS(x)			(x * TICKS_PER_SEC)	//seconds converted to values for registers
#define MILLIS(x)			(x * TICKS_PER_SEC/1000) //milliseconds converted to values for registers


/**************************************************************************************
 *						APIs supported by this driver
 *		For more information about the APIs check the function definitions in .c file
 **************************************************************************************/

//SysTick interrupt handler
void SysTick_Init (uint8_t EnOrDis);
uint8_t Systick_Read_Countflag(void);
void SysTick_Handler(void);
uint32_t check_SysTick(void);
void simple_delay(uint32_t ticks);


#endif /* INC_STM32F411XX_SYSTICK_DRIVER_H_ */
