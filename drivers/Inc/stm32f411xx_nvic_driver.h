/*
 * stm32f411xx_nvic_driver.h
 *
 *  Created on: Mar 10, 2025
 *      Author: grudz
 */

#ifndef INC_STM32F411XX_NVIC_DRIVER_H_
#define INC_STM32F411XX_NVIC_DRIVER_H_

#include "stm32f411xx.h"


void NVIC_EnableIRQ(uint8_t IRQNumber);

void NVIC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


#endif /* INC_STM32F411XX_NVIC_DRIVER_H_ */
