/*
 * scheduler_settings.c
 *
 *      Author: grudz
 */

//#include "scheduler_settings.h"

#include "stm32f411xx.h"


#define LED_GREEN	12
#define LED_ORANGE	13
#define LED_RED		14
#define LED_BLUE	15


void Periph_Init () {
	/*
	 * SysTick
	 */
	SysTick_Init(ENABLE);

	/*
	 * GPIO
	 */
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_GREEN;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_ORANGE;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_RED;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_BLUE;
	GPIO_Init(&GpioLed);

}

