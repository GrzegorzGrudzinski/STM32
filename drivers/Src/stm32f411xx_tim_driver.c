/*
 * pwm.c
 *
 *  Created on: Mar 2, 2025
 *      Author: grudz
 *
 *
 *      TODO	add function descriptions
 */

#include "stm32f411xx.h"


/******************************************************
 * 				FUNCTION DESCRIPTION
 * @fn				-	TIM_PeriClockControl
 *
 * @brief			-	This function enables or disables
 * 					peripheral clock for the given TIM
 *
 * @param pTIMx		-	base address of the gpio peripheral
 * @param EnORDis	-	ENABLE / DISABLE macros
 *
 * @return			-	none
 *
 * @Note			-	none
 *
 ********************************************************/
void TIM_PeriClockControl (TIM_RegDef_t *pTIMx, uint8_t EnORDis) {
	if(EnORDis == ENABLE)
	{
		//enable the pin
		if(pTIMx == TIM1)
			TIM1_PCLK_EN();
		else if(pTIMx == TIM2)
			TIM2_PCLK_EN();
		else if(pTIMx == TIM3)
			TIM3_PCLK_EN();
		else if(pTIMx == TIM4)
			TIM4_PCLK_EN();
		else if(pTIMx == TIM5)
			TIM5_PCLK_EN();
		else if(pTIMx == TIM9)
			TIM9_PCLK_EN();
		else if(pTIMx == TIM10)
			TIM10_PCLK_EN();
		else if(pTIMx == TIM11)
			TIM11_PCLK_EN();
	}
	else
	{
		//disable the pin
		if(pTIMx == TIM1)
			TIM1_PCLK_DI();
		else if(pTIMx == TIM2)
			TIM2_PCLK_DI();
		else if(pTIMx == TIM3)
			TIM3_PCLK_DI();
		else if(pTIMx == TIM4)
			TIM4_PCLK_DI();
		else if(pTIMx == TIM5)
			TIM5_PCLK_DI();
		else if(pTIMx == TIM9)
			TIM9_PCLK_DI();
		else if(pTIMx == TIM10)
			TIM10_PCLK_DI();
		else if(pTIMx == TIM11)
			TIM11_PCLK_DI();
	}
}


/**********************************************************
 * 					FUNCTION DESCRIPTION
 * @fn				- TIM_Init
 *
 * @brief			- Initialize the timer
 *
 * @param pTIMx		- TIM_Handle struct - basic settings of a timer
 *
 * @return			- none
 *
 * @Note			- Currently supports only PWM mode
 *
 * TODO add other tim modes
 *
 **********************************************************/
void TIM_Init(TIM_Handle_t *pTIMx) {
	/*
	 * PWM mode
	 */
	if(pTIMx->TIM_Config.TIM_Mode == TIM_MODE_PWM)
	{
		if(pTIMx->TIM_Config.TIM_ChannelENorDIS[0]) {
			pTIMx->pTIMx->CCMR1.Output.OC1M = TIM_MODE_PWM;
			pTIMx->pTIMx->CCMR1.Output.OC1PE = 1U;	//Preload enable
			pTIMx->pTIMx->CCMR1.Output.CC1S = 0U;	//Set CCx channel as output
			pTIMx->pTIMx->CCER.CC1E = 1U; //OCx enable
			pTIMx->pTIMx->CCER.CC1P = 0U; //OCx active high
		}
		if(pTIMx->TIM_Config.TIM_ChannelENorDIS[1]) {
			pTIMx->pTIMx->CCMR1.Output.OC2M = TIM_MODE_PWM;
			pTIMx->pTIMx->CCMR1.Output.OC2PE = 1U;	//Preload enable
			pTIMx->pTIMx->CCMR1.Output.CC2S = 0U;	//Set CCx channel as output
			pTIMx->pTIMx->CCER.CC2E = 1U; //OCx enable
			pTIMx->pTIMx->CCER.CC2P = 0U; //OCx active high
		}
		if(pTIMx->TIM_Config.TIM_ChannelENorDIS[2]) {
			pTIMx->pTIMx->CCMR2.Output.OC3M = TIM_MODE_PWM;
			pTIMx->pTIMx->CCMR2.Output.OC3PE = 1U;	//Preload enable
			pTIMx->pTIMx->CCMR2.Output.CC3S = 0U;	//Set CCx channel as output
			pTIMx->pTIMx->CCER.CC3E = 1U; //OCx enable
			pTIMx->pTIMx->CCER.CC3P = 1U; //OCx active high
		}
		if(pTIMx->TIM_Config.TIM_ChannelENorDIS[3]) {
			pTIMx->pTIMx->CCMR2.Output.OC4M = TIM_MODE_PWM;
			pTIMx->pTIMx->CCMR2.Output.OC4PE = 1U;	//Preload enable
			pTIMx->pTIMx->CCMR2.Output.CC4S = 0U;	//Set CCx channel as output
			pTIMx->pTIMx->CCER.CC4E = 1U; //OCx enable
			pTIMx->pTIMx->CCER.CC4P = 1U; //OCx active high
		}

		//Time base configuration
		pTIMx->pTIMx->ARR.ARR = pTIMx->TIM_Config.TIM_TB_AutoReload - 1U;
		pTIMx->pTIMx->PSC.PSC = pTIMx->TIM_Config.TIM_TB_ClockDiv - 1U;

		//
		pTIMx->pTIMx->EGR.UG = 1U;	//initialize all registers
		pTIMx->pTIMx->CR1.CEN = 1U;	//enable counter
	}
}


/*********************************************************
 * 					FUNCTION DESCRIPTION
 * @fn				- TIM_PWM_Start
 *
 * @brief			- Start the pwm modulation
 *
 * @param pTIMx		-
 * @param TIM_chan	-
 * @param procent	-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *******************************************************/
void TIM_PWM_Start(TIM_Handle_t *pTIMx, uint8_t TIM_chan, uint16_t procent) {
	pTIMx->pTIMx->CCR[TIM_chan].reg = PWM_WIDTH_PROCENT(procent);
}

/********************************************************
 * 					FUNCTION DESCRIPTION
 * @fn			-
 *
 * @brief		-
 *
 * @param		-
 *
 * @return		-
 *
 * @Note		-
 *
 *********************************************************/
//void TIM_PWM_Set_Width(TIM_Handle_t *pTIMx, uint8_t TIM_chan, uint16_t procent) {
//	pTIMx->TIM_Config.TIM_CCRx_Values[TIM_chan] =  PWM_WIDTH_PROCENT(procent);
//}

