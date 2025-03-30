/*
 * pwm.h
 *
 *  Created on: Mar 2, 2025
 *      Author: grudz
 */

#ifndef INC_STM32F411XX_TIM_DRIVER_H_
#define INC_STM32F411XX_TIM_DRIVER_H_

#include "stm32f411xx.h"


/*******************************************************
 *
 *				Structures definitions
 *
 ******************************************************/

/*
 * This is a Configuration structure for a Timer
 */
typedef struct
{
	uint8_t	 TIM_TimerNumber;        // Numer timera (np. TIM1, TIM2...)
	uint16_t TIM_TB_AutoReload;      // Wartość przepełnienia (ARR)
	uint16_t TIM_TB_ClockDiv;        // Preskaler (PSC)
	uint8_t  TIM_Mode;               // Tryb timera (np. PWM, input capture, output compare)
	uint16_t TIM_ChannelENorDIS[4];  // Początkowe wartości CCR1-CCR4
	uint16_t TIM_CCRx_Values[4];     // Początkowe wartości CCR1-CCR4
	uint8_t  TIM_INorOUT;

	/*
	uint8_t TIM_ChannelNum;         // Kanał timera (1-4)
	uint8_t TIM_ClockSource;        // Źródło zegara (wewnętrzny/zewnętrzny)
	uint8_t TIM_CounterMode;        // Tryb licznika (UP/DOWN/Center-Aligned)
	uint8_t TIM_OnePulseMode;       // Tryb jednokrotnego impulsu (One-Pulse Mode)
	uint8_t TIM_ChannelPolarity;    // Polaryzacja kanału (rising/falling edge)
	uint8_t TIM_InterruptEnable;    // Aktywacja przerwań (ENABLE/DISABLE)
	*/
} TIM_Config_t;


typedef struct
{
	//pointer to hold the base addr of the GPIO peripheral
	TIM_RegDef_t *pTIMx; 			/*!< This holds the base addr of the GPIO port to whitch the pin belongs >*/
	TIM_Config_t TIM_Config;
} TIM_Handle_t;




/**************************************************************************************
 *																					  *
 * 										MACROS										  *
 *																					  *
 **************************************************************************************/

/*
 * possible timer modes
 */
#define TIM_MODE_PWM		6

/*
 * Timer numbers
 */
#define TIM_NUM_1			1
#define TIM_NUM_2			2
#define TIM_NUM_3			3
#define TIM_NUM_4			4
#define TIM_NUM_5			5
#define TIM_NUM_9			9
#define TIM_NUM_10			10
#define TIM_NUM_11			11

/*
 * Number of the TIMx channel
 */
#define TIM_CHANNEL_1		0
#define TIM_CHANNEL_2		1
#define TIM_CHANNEL_3		2
#define TIM_CHANNEL_4		3


/*
 * % of pulse width converted to value for registers
 */
#define PWM_WIDTH_PROCENT(x)	( x * 10 )


/**************************************************************************************
 *						APIs supported by this driver
 *		For more information about the APIs check the function definitions in .c file
 **************************************************************************************/

void TIM_PeriClockControl (TIM_RegDef_t *pTIMx, uint8_t EnORDis);
void TIM_Init(TIM_Handle_t *pTIMx_Handle);
void TIM_PWM_Start(TIM_Handle_t *pTIMx_Handle, uint8_t TIM_CHANNEL, uint16_t procent);
void TIM_PWM_Set_Width(TIM_Handle_t * pTIMx_Handle, uint8_t TIM_CHANNEL, uint16_t procent);

#endif /* INC_STM32F411XX_TIM_DRIVER_H_ */
