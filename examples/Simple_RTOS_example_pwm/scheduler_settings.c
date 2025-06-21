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
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
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

	/*
	 * Timer and PWM
	 */
	TIM_Handle_t TIM_PWM;

	/*
	 * TODO - add function whitch automatically inits given pin to corresponding TIM channel
	 */

	TIM_PWM.pTIMx = TIM4;
	TIM_PWM.TIM_Config.TIM_Mode = TIM_MODE_PWM;
	TIM_PWM.TIM_Config.TIM_TB_AutoReload = 1000U;
	TIM_PWM.TIM_Config.TIM_TB_ClockDiv = 160U;
	TIM_PWM.TIM_Config.TIM_TimerNumber = TIM_NUM_4;

	TIM_PWM.TIM_Config.TIM_ChannelENorDIS[TIM_CHANNEL_1] = ENABLE;
	TIM_PWM.TIM_Config.TIM_ChannelENorDIS[TIM_CHANNEL_2] = ENABLE;
	TIM_PWM.TIM_Config.TIM_ChannelENorDIS[TIM_CHANNEL_3] = ENABLE;
	TIM_PWM.TIM_Config.TIM_ChannelENorDIS[TIM_CHANNEL_4] = ENABLE;

	TIM_PeriClockControl(TIM4, ENABLE);
	TIM_Init(&TIM_PWM);
}

