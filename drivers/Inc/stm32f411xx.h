/*
 * stm32f411xx.h
 *
 *  Created on: Jul 22, 2024
 *      Author: grudz
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

#include "registers_bits/tim_register_bits.h"
#include "registers_bits/adc_register_bits.h"
#include "registers_bits/spi_register_bits.h"

/*	Generic Macros*/
#define __vo				volatile
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define HIGH				1
#define LOW					0
#define FLAG_SET			SET
#define FLAG_RESET			RESET


/************************************START:Processor Specific Details*********************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0			( ( __vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1			( ( __vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2			( ( __vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3			( ( __vo uint32_t*) 0xE000E10C )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0			( ( __vo uint32_t*) 0XE000E180 )
#define NVIC_ICER1			( ( __vo uint32_t*) 0XE000E184 )
#define NVIC_ICER2			( ( __vo uint32_t*) 0XE000E188 )
#define NVIC_ICER3			( ( __vo uint32_t*) 0XE000E18C )

/*
 * ARM Cortex Mx Processor Priority register Addresses calculation
 */
#define NVIC_PR_BASE_ADDR	( ( __vo uint32_t*) 0xE000E400 )

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED		4


/*******************************************************
 *
 *					BASE ADDRESSES
 *
 ******************************************************/

/*	Base addresses of memory	*/
#define FLASH_BASEADDR		0x08000000U		//Base addr of flash memory
#define SRAM1_BASEADDR		0x20000000U		//Base addr of SRAM (128Kb)
//#define SRAM2_BASEADDR 	//nie ma
#define SRAM				SRAM1__BASEADDR
#define ROM_BASEADDR		0x1FFF0000U		//base addr of system memory

/*	Base addresses of bus domains	*/
#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE		//TIM2 PERIPH (offset 0x00 TIMx_CR1); BASE ADDRESS OF PERIPHERAL BASE (APB1)
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/*	Base addresses of peripherals hanging on AHB1 bus	*/
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

/*	Base addresses of peripherals hanging on APB1 bus	*/
#define	I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define	I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define	I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)

#define TIM2_BASEADDR		(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR		(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR		(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR		(APB1PERIPH_BASE + 0x0C00)

#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)

/*	Base addresses of peripherals hanging on APB2 bus	*/
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)

#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASE + 0x3400)
#define SPI5_BASEADDR		(APB2PERIPH_BASE + 0x5000)

#define TIM1_BASEADDR		(APB2PERIPH_BASE + 0x0000)
#define TIM9_BASEADDR		(APB2PERIPH_BASE + 0x4000)
#define TIM10_BASEADDR		(APB2PERIPH_BASE + 0x4400)
#define TIM11_BASEADDR		(APB2PERIPH_BASE + 0x4800)

#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)

#define ADC1_BASEADDR		(APB2PERIPH_BASE + 0x2000)

/* SysTick	*/
#define SYSTICK_BASEADDR 	(0xE000E010U)



/*******************************************************
 *
 *			PERIPHERAL REGISTERS STRUCTURES
 *
 ******************************************************/

/*******************************************************************************************
								SysTic registers
	The processor has a 24-bit system timer, SysTick, that counts down from the reload value to
	zero, reloads (wraps to) the value in the STK_LOAD register on the next clock edge, then
	counts down on subsequent clocks

	Ensure software uses aligned word accesses to access the SysTick registers.
	The SysTick counter reload and current value are undefined at reset, the correct
	initialization sequence for the SysTick counter is:
	 1. Program reload value.
	 2. Clear current value.
	 3. Program Control and Status register
 ********************************************************************************************/
typedef struct
{
	__vo uint32_t STK_CTRL; 	//0x00 - Control and status register
	__vo uint32_t STK_LOAD; 	//0x04 - Reload value
	__vo uint32_t STK_VAL;		//0x08 - Current value
	__vo uint32_t STK_CALIB;	//0x0C - Calibration value

} SysTick_RegDef_t;

/*******************************************************************************************
  									GPIO registers
	Each general-purpose I/O port has four 32-bit configuration registers (GPIOx_MODER,
	GPIOx_OTYPER, GPIOx_OSPEEDR and GPIOx_PUPDR), two 32-bit data registers
	(GPIOx_IDR and GPIOx_ODR), a 32-bit set/reset register (GPIOx_BSRR), a 32-bit locking
	register (GPIOx_LCKR) and two 32-bit alternate function selection register (GPIOx_AFRH
	and GPIOx_AFRL).

	The GPIO registers can be accessed by byte (8 bits), half-words (16 bits) or words (32 bits).
 ********************************************************************************************/
typedef struct
{
	__vo uint32_t MODER;		//GPIO port mode register				0x00
	__vo uint32_t OTYPER;		//GPIO port output type register		0x04
	__vo uint32_t OSPEEDR;		//GPIO port output speed register		0x08
	__vo uint32_t PUPDR;		//GPIO port pull-up/pull-down register	0x0C
	__vo uint32_t IDR;			//GPIO port input data register			0x10
	__vo uint32_t ODR;			//GPIO port output data register		0x14
	__vo uint32_t BSRR;			//GPIO port bit set/reset register		0x18
	__vo uint32_t LCKR;			//GPIO port configuration lock register	0x1C
	__vo uint32_t AFR[2];		//GPIO alternate function low/high register	( [0] = AFRL	0x20; [1] = AFRH 	0x24)
} GPIO_RegDef_t;


/*******************************************************************************************
  									SPI registers
The SPI interface provides two main functions, supporting either the SPI protocol or the I2S
audio protocol. By default, it is the SPI function that is selected. It is possible to switch the
interface from SPI to I2S by software.
The serial peripheral interface (SPI) allows half/ full-duplex, synchronous, serial
communication with external devices. The interface can be configured as the master and in
this case it provides the communication clock (SCK) to the external slave device. The
interface is also capable of operating in multimaster configuration.

 ********************************************************************************************/
/*
 *Old version
	typedef struct
	{
		__vo uint32_t CR1;				//0x00	SPI control register 1 (SPI_CR1)(not used in I2S mode)
		__vo uint32_t CR2;				//0x04	SPI control register 2
		__vo uint32_t SR;				//0x08	SPI status register
		__vo uint32_t DR;				//0x0C	SPI data register
		__vo uint32_t CRCPR;			//0x10	SPI CRC polynomial register (SPI_CRCPR)(not used in I2S	mode)
		__vo uint32_t RXCRCR;			//0x14	SPI RX CRC register (SPI_RXCRCR)(not used in I2S mode)
		__vo uint32_t TXCRCR;			//0x18	SPI TX CRC register (SPI_TXCRCR)(not used in I2S mode)
		__vo uint32_t I2SCFGR;			//0x1C	SPI_I2S configuration register
		__vo uint32_t I2SPR;			//0x20	SPI_I2S prescaler register
	} SPI_RegDef_t;
*/

typedef struct
{
	union SPI_CR1_t CR1;			//0x00	SPI control register 1 (SPI_CR1)(not used in I2S mode)
	union SPI_CR2_t CR2;			//0x04	SPI control register 2
	union SPI_SR_t SR;				//0x08	SPI status register
	union SPI_DR_t DR;				//0x0C	SPI data register
	union SPI_CRCPR_t CRCPR;		//0x10	SPI CRC polynomial register (SPI_CRCPR)(not used in I2S	mode)
	union SPI_RXCRCR_t RXCRCR;		//0x14	SPI RX CRC register (SPI_RXCRCR)(not used in I2S mode)
	union SPI_TXCRCR_t TXCRCR;		//0x18	SPI TX CRC register (SPI_TXCRCR)(not used in I2S mode)
	union SPI_I2SCFGR_t I2SCFGR;	//0x1C	SPI_I2S configuration register
	union SPI_I2SPR_t I2SPR;		//0x20	SPI_I2S prescaler register
} SPI_RegDef_t;

/*******************************************************************************************
 								RCC Clock registers

 ********************************************************************************************/
typedef struct
{
	__vo uint32_t CR;				//0x00	RCC clock control register
	__vo uint32_t PLLFGR;			//0x04	RCC PLL configuration register
	__vo uint32_t CFGR;				//0x08	RCC clock configuration register
	__vo uint32_t CIR;				//0x0C	RCC clock interrupt register
	__vo uint32_t AHB1RSTR;			//0x10	RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;			//0x14	RCC AHB2 peripheral reset register
	uint32_t RESERVED0[2];			//0x18;0x1C
	__vo uint32_t APB1RSTR;			//0x20	RCC AHB1 peripheral reset register
	__vo uint32_t APB2RSTR;			//0x24	RCC AHB2 peripheral reset register
	uint32_t RESERVED1[2];			//0x28;0x2C
	__vo uint32_t AHB1ENR;			//0x30	RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;			//0x34	RCC AHB2 peripheral clock enable register
	uint32_t RESERVED2[2];			//0x38;0x3C
	__vo uint32_t APB1ENR;			//0x40	RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;			//0x44	RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2];			//0x48;0x4C
	__vo uint32_t AHB1LPENR;		//0x50	RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;		//0x54	RCC AHB2 peripheral clock enable in low power mode register
	uint32_t RESERVED4[2];			//0x58;0x5C
	__vo uint32_t APB1LPENR;		//0x60	RCC APB1 peripheral clock enabled in low power mode register
	__vo uint32_t APB2LPENR;		//0x64	RCC APB2 peripheral clock enabled in low power mode register
	uint32_t RESERVED5[2];			//0x68;0x6C
	__vo uint32_t BDCR;				//0x70	RCC Backup domain control register
	__vo uint32_t CSR;				//0x74	RCC clock control & status register
	uint32_t RESERVED6[2];			//0x78;0x7C
	__vo uint32_t SSCGR;			//0x80	RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFG;		//0x84	RCC PLLI2S configuration register
	uint32_t RESERVED7;				//0x88
	__vo uint32_t DCKCFGR;			//0x8C	RCC Dedicated Clocks Configuration Register
} RCC_RegDef_t;

/*******************************************************************************************
 									EXTI registers
	The external interrupt/event controller consists of up to 23 edge detectors for generating
	event/interrupt requests. Each input line can be independently configured to select the type
	(interrupt or event) and the corresponding trigger event (rising or falling or both). Each line
	can also masked independently. A pending register maintains the status line of the interrupt
	requests
 ********************************************************************************************/
typedef struct
{
	__vo uint32_t IMR;				//0x00
	__vo uint32_t EMR;				//0x04
	__vo uint32_t RTSR;				//0x08
	__vo uint32_t FTSR;				//0x0C
	__vo uint32_t SWIER;			//0x10
	__vo uint32_t PR;				//0x14
} EXTI_RegDef_t;

/*******************************************************************************************
  									SYSCFG registers
 	 The system configuration controller is mainly used to remap the memory accessible in the
	code area and manage the external interrupt line connection to the GPIOs
 ********************************************************************************************/

typedef struct
{
	__vo uint32_t MEMRMP;	 		//0x00	SYSCFG memory remap register
	__vo uint32_t PMC;		 		//0x04	SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4]; 		//0x08 - 0x14	SYSCFG external interrupt configuration register
	uint32_t RESERVED[2];			//0x18-0x1C
	__vo uint32_t CMPCR;		 	//0x20	Compensation cell control register
} SYSCFG_RegDef_t;


/*******************************************************************************************
  									TIM registers
	The general-purpose timers consist of a 16-bit or 32-bit auto-reload counter driven by a
	programmable prescaler.
 	They may be used for a variety of purposes, including measuring the pulse lengths of input
	signals (input capture) or generating output waveforms (output compare and PWM).
 ********************************************************************************************/
typedef struct { //registers are defined in pwm.h file
	union TIMx_CR1_t CR1;		//0x00
	union TIMx_CR2_t CR2;		//0x04
	union TIMx_SMCR_t SMCR;		//0x08
	union TIMx_DIER_t DIER;		//0x0C
	union TIMx_SR_t SR;			//0x10
	union TIMx_EGR_t EGR;		//0x14
	union TIMx_CCMR1_t CCMR1;	//0x18
	union TIMx_CCMR2_t CCMR2;	//0x1C
	union TIMx_CCER_t CCER;		//0x20
	union TIMx_CNT_t CNT;		//0x24
	union TIMx_PSC_t PSC;		//0x28
	union TIMx_ARR_t ARR;		//0x2C
	uint32_t RESERVED1;			//0x30
	union TIMx_CCR_t CCR[4];	//0x34 - 0x40; 0-CCR1, 1-CCR2...
	uint32_t RESERVED2;			//0x44
	union TIMx_DCR_t DCR;		//0x48
	union TIMx_DMAR_t DMAR;		//0x4C
	union TIMx_OR_t OR;			//0x50
} TIM_RegDef_t;


/*******************************************************************************************
						Analog-to-digital converter (ADC) registers
	The 12-bit ADC is a successive approximation analog-to-digital converter. It has up to 19
	multiplexed channels allowing it to measure signals from 16 external sources, two internal
	sources, and the VBAT channel. The A/D conversion of the channels can be performed in
	single, continuous, scan or discontinuous mode. The result of the ADC is stored into a left-
	or right-aligned 16-bit data register.
 	 The analog watchdog feature allows the application to detect if the input voltage goes
	beyond the user-defined, higher or lower thresholds.
 ********************************************************************************************/
typedef struct {
	union ADC_SR_t 	SR;			//0x00
	union ADC_CR1_t CR1;		//0x04
	union ADC_CR2_t CR2;		//0x08
	union ADC_SMPR1_t SMPR1;	//0x0C
	union ADC_SMPR2_t SMPR2;	//0x10
	union ADC_JOFRx_t JOFR[4];	//0x14 - 0x20
	union ADC_HTR_t HTR;		//0x24
	union ADC_LTR_t LTR;		//0x28
	union ADC_SQR1_t SQR1;		//0x2C
	union ADC_SQR2_t SQR2;		//0x30/
	union ADC_SQR3_t SQR3;		//0x34
	union ADC_JSQR_t JSQR;		//0x38
	union ADC_JDR_t JDR[4];		//0x3C - 0x48
	union ADC_DR_t 	DR;			//0x4C

/*	uint32_t RESERVED[181] - */
//	union ADR_CCR CCR; 			//0x300	- ADC common reg
} ADC_RegDef_t;


/*******************************************************
 *
 *			PERIPHERAL DEFINITIONS
 *
 ******************************************************/

/*	Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)	*/
#define GPIOA				((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH			 	((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0 :\
									  (x == GPIOB) ? 1 :\
									  (x == GPIOC) ? 2 :\
									  (x == GPIOD) ? 3 :\
									  (x == GPIOE) ? 4 :\
									  (x == GPIOH) ? 7 : 0 )

#define RCC					((RCC_RegDef_t*)	 RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)	 EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)	 SYSCFG_BASEADDR)
#define SysTick				((SysTick_RegDef_t*) SYSTICK_BASEADDR)

#define SPI1				((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*) SPI4_BASEADDR)
#define SPI5				((SPI_RegDef_t*) SPI5_BASEADDR)


#define TIM1				((TIM_RegDef_t*) TIM1_BASEADDR)
#define TIM2				((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM3				((TIM_RegDef_t*) TIM3_BASEADDR)
#define TIM4				((TIM_RegDef_t*) TIM4_BASEADDR)
#define TIM5				((TIM_RegDef_t*) TIM5_BASEADDR)
#define TIM9				((TIM_RegDef_t*) TIM9_BASEADDR)
#define TIM10				((TIM_RegDef_t*) TIM10_BASEADDR)
#define TIM11				((TIM_RegDef_t*) TIM11_BASEADDR)

#define ADC1				( (ADC_RegDef_t*) ADC1_BASEADDR)

/*******************************************************
 *
 *			CLOCK ENABLE MACROS
 *
 ******************************************************/
/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0 ) )	/*GPIOA_PERI_CLOCK_ENABLE / PA_PCLK_EN	*/
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7 ) )

/* Clock Enable Macros for SPIx peripherals */
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15 ) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 13 ) )
#define SPI5_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 20 ) )

/* Clock Enable Macros for I2Cx peripherals */
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23 ) )

/* Clock Enable Macros for USARTx peripherals */
#define USART1_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 5 ) )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 17 ) )

/* Clock Enable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 14 ) )

/* Clock Enable Macros for TIMx peripherals */
#define TIM1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 0 ) )
#define TIM2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 0 ) )
#define TIM3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 1 ) )
#define TIM4_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 2 ) )
#define TIM5_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 3 ) )
#define TIM9_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 16 ) )
#define TIM10_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 17 ) )
#define TIM11_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 18 ) )

/* Clock Enable Macros for ADC */
#define ADC1_PCLK_EN()		( RCC->APB2ENR |= (1U << 8) )

/*******************************************************
 *
 *			CLOCK DISNABLE MACROS
 *
 ******************************************************/

/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0 ) )	//GPIOA_PERI_CLOCK_ENABLE / PA_PCLK_EN
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7 ) )

/* Clock Disable Macros for SPIx peripherals */
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 13 ) )
#define SPI5_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 20 ) )

/* Clock Disable Macros for I2Cx peripherals */
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ) )

/* Clock Disable Macros for USARTx peripherals */
#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 5 ) )
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~( 1 << 17 ) )

/* Clock Disable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 14 ) )

/* Clock Disable Macros for TIMx peripherals */
#define TIM1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 0 ) )
#define TIM2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 0 ) )
#define TIM3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 1 ) )
#define TIM4_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 2 ) )
#define TIM5_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 3 ) )
#define TIM9_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 16 ) )
#define TIM10_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 17 ) )
#define TIM11_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 18 ) )


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 0 ) );	( RCC->AHB1RSTR &= ~( 1 << 0 ) ); }while(0)
#define GPIOB_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 1 ) );	( RCC->AHB1RSTR &= ~( 1 << 1 ) ); }while(0)
#define GPIOC_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 2 ) );	( RCC->AHB1RSTR &= ~( 1 << 2 ) ); }while(0)
#define GPIOD_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 3 ) );	( RCC->AHB1RSTR &= ~( 1 << 3 ) ); }while(0)
#define GPIOE_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 4 ) );	( RCC->AHB1RSTR &= ~( 1 << 4 ) ); }while(0)
#define GPIOH_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 7 ) );	( RCC->AHB1RSTR &= ~( 1 << 7 ) ); }while(0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 12 ) );	( RCC->AHB1RSTR &= ~( 1 << 12 ) ); }while(0)
#define SPI2_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 14 ) );	( RCC->AHB1RSTR &= ~( 1 << 14 ) ); }while(0)
#define SPI3_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 15 ) );	( RCC->AHB1RSTR &= ~( 1 << 15 ) ); }while(0)
#define SPI4_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 13 ) );	( RCC->AHB1RSTR &= ~( 1 << 13 ) ); }while(0)
#define SPI5_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 20 ) );	( RCC->AHB1RSTR &= ~( 1 << 20 ) ); }while(0)

/*
 * Macros to reset TIMx
 */
#define TIM1_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 0 ) );	( RCC->AHB1RSTR &= ~( 1 << 0 ) ); }while(0)
#define TIM2_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 0 ) );	( RCC->AHB1RSTR &= ~( 1 << 0 ) ); }while(0)
#define TIM3_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 1 ) );	( RCC->AHB1RSTR &= ~( 1 << 1 ) ); }while(0)
#define TIM4_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 2 ) );	( RCC->AHB1RSTR &= ~( 1 << 2 ) ); }while(0)
#define TIM5_REG_RESET()	do{( RCC->APB1RSTR |= ( 1 << 3 ) );	( RCC->AHB1RSTR &= ~( 1 << 3 ) ); }while(0)
#define TIM9_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 16 ) );( RCC->AHB1RSTR &= ~( 1 << 16 ) ); }while(0)
#define TIM10_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 17 ) );( RCC->AHB1RSTR &= ~( 1 << 17 ) ); }while(0)
#define TIM11_REG_RESET()	do{( RCC->APB2RSTR |= ( 1 << 18 ) );( RCC->AHB1RSTR &= ~( 1 << 18 ) ); }while(0)





/*
 * IRQ (Interrupt Request) numbers
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_ADC			18

#define IRQ_NO_EXTI_AUTO(x)		( (x == 0) ? 6 :\
						  	  	  (x == 1) ? 7 :\
						  	  	  (x == 2) ? 8 :\
						  	  	  (x == 3) ? 9 :\
						  	  	  (x == 4) ? 10 :\
						  	  	  ((x >= 5) && (x <= 9)) ? 23 :\
						  	  	  ((x >= 10) && (x <= 15)) ? 40 : 6 )

/*
 * IRQ priority levels
 */
#define NVIC_IRQ_PRIO_0		0
#define NVIC_IRQ_PRIO_1		1
#define NVIC_IRQ_PRIO_2		2
#define NVIC_IRQ_PRIO_3		3
#define NVIC_IRQ_PRIO_4		4
#define NVIC_IRQ_PRIO_5		5
#define NVIC_IRQ_PRIO_6		6
#define NVIC_IRQ_PRIO_7		7
#define NVIC_IRQ_PRIO_8		8
#define NVIC_IRQ_PRIO_9		9
#define NVIC_IRQ_PRIO_10	10
#define NVIC_IRQ_PRIO_11	11
#define NVIC_IRQ_PRIO_12	12
#define NVIC_IRQ_PRIO_13	13
#define NVIC_IRQ_PRIO_14	14
#define NVIC_IRQ_PRIO_15	15


#define ADC_IRQ_PRIO		25 // 0x0000 0088
/*******************************************************
 *
 *					INCLUDES
 *
 ******************************************************/
#include "stm32f411xx_nvic_driver.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_systick_driver.h"
#include "stm32f411xx_tim_driver.h"
#include "stm32f411xx_adc_driver.h"
#include "stm32f411xx_spi_driver.h"

//#include <mcu_stm32f411xx_spi_driver.h>


#endif /* INC_STM32F411XX_H_ */
