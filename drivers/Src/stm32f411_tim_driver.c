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


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-	TIM_PeriClockControl
 *
 * @brief					-	This function enables or disables peripheral clock for the given TIM
 *
 * @param[in]				-	base address of the gpio peripheral
 * @param[in]				-	ENABLE / DISABLE macros
 *
 * @return					-	none
 *
 * @Note					-	none
 *
 ******************************************************************/
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


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						- TIM_Init
 *
 * @brief					- Initialize the timer
 *
 * @param[in]				- TIM_Handle struct - basic settings of a timer
 *
 * @return					- none
 *
 * @Note					- Currently supports only PWM mode
 *
 * TODO add other tim modes
 *
 ******************************************************************/
void TIM_Init(TIM_Handle_t *pTIMx_Handle) {
	/*
	 * PWM mode
	 */
	if(pTIMx_Handle->TIM_Config.TIM_Mode == TIM_MODE_PWM)
	{
		if(pTIMx_Handle->TIM_Config.TIM_ChannelENorDIS[0]) {
			pTIMx_Handle->pTIMx->CCMR1.Output.OC1M = TIM_MODE_PWM;
			pTIMx_Handle->pTIMx->CCMR1.Output.OC1PE = 1U;	//Preload enable
			pTIMx_Handle->pTIMx->CCMR1.Output.CC1S = 0U;	//Set CCx channel as output
			pTIMx_Handle->pTIMx->CCER.CC1E = 1U; //OCx enable
			pTIMx_Handle->pTIMx->CCER.CC1P = 0U; //OCx active high
		}
		if(pTIMx_Handle->TIM_Config.TIM_ChannelENorDIS[1]) {
			pTIMx_Handle->pTIMx->CCMR1.Output.OC2M = TIM_MODE_PWM;
			pTIMx_Handle->pTIMx->CCMR1.Output.OC2PE = 1U;	//Preload enable
			pTIMx_Handle->pTIMx->CCMR1.Output.CC2S = 0U;	//Set CCx channel as output
			pTIMx_Handle->pTIMx->CCER.CC2E = 1U; //OCx enable
			pTIMx_Handle->pTIMx->CCER.CC2P = 0U; //OCx active high
		}
		if(pTIMx_Handle->TIM_Config.TIM_ChannelENorDIS[2]) {
			pTIMx_Handle->pTIMx->CCMR2.Output.OC3M = TIM_MODE_PWM;
			pTIMx_Handle->pTIMx->CCMR2.Output.OC3PE = 1U;	//Preload enable
			pTIMx_Handle->pTIMx->CCMR2.Output.CC3S = 0U;	//Set CCx channel as output
			pTIMx_Handle->pTIMx->CCER.CC3E = 1U; //OCx enable
			pTIMx_Handle->pTIMx->CCER.CC3P = 1U; //OCx active high
		}
		if(pTIMx_Handle->TIM_Config.TIM_ChannelENorDIS[3]) {
			pTIMx_Handle->pTIMx->CCMR2.Output.OC4M = TIM_MODE_PWM;
			pTIMx_Handle->pTIMx->CCMR2.Output.OC4PE = 1U;	//Preload enable
			pTIMx_Handle->pTIMx->CCMR2.Output.CC4S = 0U;	//Set CCx channel as output
			pTIMx_Handle->pTIMx->CCER.CC4E = 1U; //OCx enable
			pTIMx_Handle->pTIMx->CCER.CC4P = 1U; //OCx active high
		}

		//Time base configuration
		pTIMx_Handle->pTIMx->ARR.ARR = pTIMx_Handle->TIM_Config.TIM_TB_AutoReload - 1U;
		pTIMx_Handle->pTIMx->PSC.PSC = pTIMx_Handle->TIM_Config.TIM_TB_ClockDiv - 1U;

		//
		pTIMx_Handle->pTIMx->EGR.UG = 1U;	//initialize all registers
		pTIMx_Handle->pTIMx->CR1.CEN = 1U;	//enable counter
	}
}


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						- TIM_PWM_Start
 *
 * @brief					- Start the pwm modulation
 *
 * @param[in]				-
 * @param[in]				-
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 ******************************************************************/
void TIM_PWM_Start(TIM_Handle_t *pTIMx_Handle, uint8_t TIM_CHANNEL, uint16_t procent) {
	pTIMx_Handle->pTIMx->CCR[TIM_CHANNEL].reg = PWM_WIDTH_PROCENT(procent);
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
//void TIM_PWM_Set_Width(TIM_Handle_t *pTIMx_Handle, uint8_t TIM_CHANNEL, uint16_t procent) {
//	pTIMx_Handle->TIM_Config.TIM_CCRx_Values[TIM_CHANNEL] =  PWM_WIDTH_PROCENT(procent);
//}

