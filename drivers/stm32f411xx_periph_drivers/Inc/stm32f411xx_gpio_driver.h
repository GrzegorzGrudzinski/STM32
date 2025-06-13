/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Jul 26, 2024
 *      Author: grudz
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOPType;			/*!< possible values from @GPIO_PIN_OPTYPE >*/
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	//pointer to hold the base addr of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; 			/*!< This holds the base addr of the GPIO port to whitch the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;



/**************************************************************************************
 *																					  *
 * 										MACROS										  *
 *																					  *
 **************************************************************************************/
/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * possible modes
 */
//non-interrupt modes	(<=3)
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
//interrupt modes		(>3)
#define GPIO_MODE_IT_FT		4	//INPUT FALLING EDGE
#define GPIO_MODE_IT_RT		5	//INPUT RISING EDGE
#define GPIO_MODE_IT_RFT	6	//INPUT RISING / FALLING EDGE TRIGGER

/*
 * @GPIO_PIN_OPTYPE
 * possible output types
 */
#define GPIO_OP_TYPE_PP		0	//output - push pull
#define GPIO_OP_TYPE_OD		1	//output - open drain

/*
 * @GPIO_PIN_SPEED
 * output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD
 * pull up / pull down configurations
 */
#define GPIO_NO_PUPD		0	//no pull up/down
#define GPIO_PIN_PU			1	//pull up
#define GPIO_PIN_PD			2	//pull down




/**************************************************************************************
 *						APIs supported by this driver
 *		For more information about the APIs check the function definitions in .c file
 **************************************************************************************/

//Peripheral Clock setup
void 		GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnORDis); // enable/disable the peripheral clock for given gpio baseaddr

//Init and De-init
void 		GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void 		GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Data read and write
uint8_t		GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t	GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);	//16 pinow
void 		GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void 		GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void 		GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration and ISR handling
void	 	GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnORDis);
void 		GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void		GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
