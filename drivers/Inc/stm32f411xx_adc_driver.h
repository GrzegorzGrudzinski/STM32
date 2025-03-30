/*
 * stm32f411xx_adc_driver.h
 *
 *  Created on: Mar 5, 2025
 *      Author: grudz
 */

#ifndef SRC_STM32F411XX_ADC_DRIVER_H_
#define SRC_STM32F411XX_ADC_DRIVER_H_

#include "stm32f411xx.h"

typedef struct {
	GPIO_RegDef_t *pGPIOx;
	uint8_t Pin_Number;
	uint8_t ADC_RegOrInj;
	uint8_t ADC_Mode;
	uint8_t ADC_Conversion_Enable[16];
} ADC_Config_t;

typedef struct {
	ADC_RegDef_t *pADCx;
	ADC_Config_t ADC_Config_t;
} ADC_Handle_t;


/**************************************************************************************
 *																					  *
 * 										MACROS										  *
 *																					  *
 **************************************************************************************/

/*
 * ADC conversion type
 */
#define ADC_CONV_REGULAR		0
#define ADC_CONV_INJECTED		1

/*
 * ADC mode
 */
#define ADC_MODE_SINGLE_CONV	0
#define ADC_MODE_CONT_CONV		1
#define ADC_MODE_DISCONT_CONV	2
#define ADC_MODE_FAST_CONV		3
#define ADC_MODE_SCAN			4
#define ADC_MODE_AUTO_INJ		5
#define ADC_MODE_INJ_GROUP		6

/*
 * ADC resolution
 */
#define ADC_RES_12_BIT			0
#define ADC_RES_10_BIT			1
#define ADC_RES_8_BIT			2
#define ADC_RES_6_BIT			3

#define ADC_12BIT_MAX_VAL		4095

/**************************************************************************************
 *						APIs supported by this driver
 *		For more information about the APIs check the function definitions
 **************************************************************************************/

/*
 *
 */
void ADC_Init(ADC_Handle_t *pADCx);
uint16_t ADC_Read(ADC_Handle_t *pADCx, uint8_t range);




#endif /* SRC_STM32F411XX_ADC_DRIVER_H_ */
