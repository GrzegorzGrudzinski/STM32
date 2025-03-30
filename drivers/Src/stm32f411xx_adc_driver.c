/*
 * stm32f411xx_adc_driver.c
 *
 *  Created on: Mar 5, 2025
 *      Author: grudz
 *
 *      TODO	add function descriptions
 *
 */

#include "stm32f411xx_adc_driver.h"



/*
 * global variable for adc output
 */
static uint16_t volatile adcValue;




/**************************************************************************************
 *																					  *
 * 									FUNCTIONS										  *
 *																					  *
 **************************************************************************************/


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						- ADC_Init
 *
 * @brief					- Initialize ADC
 *
 * @param[in]				- ADC_Handle_t struct - base settings of an ADC
 *
 * @return					- none
 *
 * @Note					- Currently supports single and continous
 * 							  conversion modes of regular channels
 *
 *	TODO finish other conversion modes
 ******************************************************************/
void ADC_Init(ADC_Handle_t *pADCx){
	//RCC clock enable
	GPIO_PeriClockControl(pADCx->ADC_Config_t.pGPIOx, ENABLE);
	ADC1_PCLK_EN();

	pADCx->ADC_Config_t.pGPIOx->MODER |= (GPIO_MODE_ANALOG << (2*pADCx->ADC_Config_t.Pin_Number));

	// ADC clear and disable before configuration
	pADCx->pADCx->CR2.reg = 0U;

	//	ADC resolution
	pADCx->pADCx->CR1.RES = ADC_RES_12_BIT;

	if (pADCx->ADC_Config_t.ADC_RegOrInj) { /* Injected	 */
		uint8_t seq_length = 0;
		for(int i=0; i<=15; ++i) {
			if (pADCx->ADC_Config_t.ADC_Conversion_Enable[i] != 0) {
				seq_length++;
			}
		}
		//number of conversions
		pADCx->pADCx->JSQR.JL = seq_length;
		/* Enable selected channels */
		pADCx->pADCx->JSQR.JSQ1 = pADCx->ADC_Config_t.ADC_Conversion_Enable[0];
		pADCx->pADCx->JSQR.JSQ2 = pADCx->ADC_Config_t.ADC_Conversion_Enable[1];
		pADCx->pADCx->JSQR.JSQ3 = pADCx->ADC_Config_t.ADC_Conversion_Enable[2];
		pADCx->pADCx->JSQR.JSQ4 = pADCx->ADC_Config_t.ADC_Conversion_Enable[3];

		switch (pADCx->ADC_Config_t.ADC_Mode) {
		case ADC_MODE_SINGLE_CONV: /* Single conversion mode */
			break;
		case ADC_MODE_DISCONT_CONV:
			//	TODO
			break;
		case ADC_MODE_FAST_CONV:
			//	TODO
			break;
		case ADC_MODE_SCAN:
			//	TODO
			break;
		case ADC_MODE_AUTO_INJ:
			//	TODO
			break;
		default:
			break;
		}
		//ADC on
		pADCx->pADCx->CR2.ADON = 1;
		//ADC start conversion
		pADCx->pADCx->CR2.JSWSTART = 1;
	}
	else {		/* Regular */
		uint8_t seq_length = 0;
		for(int i=0; i<=15; ++i) {
			if (pADCx->ADC_Config_t.ADC_Conversion_Enable[i] != 0) {
				seq_length++;
			}
		}
		//number of conversions
		pADCx->pADCx->SQR1.L = seq_length;
		/* Enable selected channels */
		pADCx->pADCx->SQR3.SQ1 = pADCx->ADC_Config_t.ADC_Conversion_Enable[0];
		pADCx->pADCx->SQR3.SQ2 = pADCx->ADC_Config_t.ADC_Conversion_Enable[1];
		pADCx->pADCx->SQR3.SQ3 = pADCx->ADC_Config_t.ADC_Conversion_Enable[2];
		pADCx->pADCx->SQR3.SQ4 = pADCx->ADC_Config_t.ADC_Conversion_Enable[3];
		pADCx->pADCx->SQR3.SQ5 = pADCx->ADC_Config_t.ADC_Conversion_Enable[4];
		pADCx->pADCx->SQR3.SQ6 = pADCx->ADC_Config_t.ADC_Conversion_Enable[5];
		pADCx->pADCx->SQR2.SQ7  = pADCx->ADC_Config_t.ADC_Conversion_Enable[6];
		pADCx->pADCx->SQR2.SQ8  = pADCx->ADC_Config_t.ADC_Conversion_Enable[7];
		pADCx->pADCx->SQR2.SQ9  = pADCx->ADC_Config_t.ADC_Conversion_Enable[8];
		pADCx->pADCx->SQR2.SQ10 = pADCx->ADC_Config_t.ADC_Conversion_Enable[9];
		pADCx->pADCx->SQR2.SQ11 = pADCx->ADC_Config_t.ADC_Conversion_Enable[10];
		pADCx->pADCx->SQR2.SQ12 = pADCx->ADC_Config_t.ADC_Conversion_Enable[11];
		pADCx->pADCx->SQR1.SQ13 = pADCx->ADC_Config_t.ADC_Conversion_Enable[12];
		pADCx->pADCx->SQR1.SQ14 = pADCx->ADC_Config_t.ADC_Conversion_Enable[13];
		pADCx->pADCx->SQR1.SQ15 = pADCx->ADC_Config_t.ADC_Conversion_Enable[14];
		pADCx->pADCx->SQR1.SQ16 = pADCx->ADC_Config_t.ADC_Conversion_Enable[15];

		switch (pADCx->ADC_Config_t.ADC_Mode) {
		case ADC_MODE_SINGLE_CONV: /* Single conversion mode */
			break;
		case ADC_MODE_CONT_CONV: /* Continuous conversion mode */
			pADCx->pADCx->CR1.EOCIE = 1;
			pADCx->pADCx->CR2.CONT = 1;

			NVIC_EnableIRQ(IRQ_NO_ADC);
			NVIC_IRQPriorityConfig(IRQ_NO_ADC, ADC_IRQ_PRIO);
			//set priority

			//In continuous conversion mode, the ADC starts a new conversion as soon as it finishes one.
			//This mode is started with the CONT bit at 1 either by external trigger or by setting the
			//SWSTRT bit in the ADC_CR2 register (for regular channels only)
			break;
		case ADC_MODE_DISCONT_CONV:
			//	TODO
			break;
		case ADC_MODE_FAST_CONV:
			//	TODO
			break;
		case ADC_MODE_SCAN:
			//	TODO
			break;
		default:
			break;
		}

		/*
		 *
		 */
		//ADC on
		pADCx->pADCx->CR2.ADON = 1;

		pADCx->pADCx->CR2.SWSTART = 1;
	}

	//	TODO
    // Short delay for safety
    for (int i = 0; i < 5000; i++);

}


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						- ADC_Read
 *
 * @brief					- read the value from ADC channel
 * 							  and convert it to the given range
 *
 * @param[in]				- ADC handle struct - basic settings of ADC
 *
 * @return					- value converted to the given range
 *
 * @Note					- Polling function (only in single conversion mode), might affect performance
 *
 *	TODO	add injected mode
 *	TODO	add interrupt to single conv. mode
 ******************************************************************/
uint16_t ADC_Read(ADC_Handle_t *pADCx, uint8_t range) {
	switch (pADCx->ADC_Config_t.ADC_Mode) {
	case ADC_MODE_SINGLE_CONV:
		pADCx->pADCx->CR2.SWSTART = 1;
		while(!(pADCx->pADCx->SR.EOC)){;}
		adcValue = ((pADCx->pADCx->DR.reg));

		return ( (adcValue*range) / ADC_12BIT_MAX_VAL);
		break;
	case ADC_MODE_CONT_CONV:
		/*	adcValue = ((pADCx->pADCx->DR.reg));  */
		return ( (adcValue*range) / ADC_12BIT_MAX_VAL);
		break;
	default:
		/*	error	*/
		return 0;
		break;
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
void ADC_IRQHandler(void) {
	if(ADC1->SR.EOC) {
		adcValue = ADC1->DR.reg;
	}
}

