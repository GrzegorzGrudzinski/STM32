/*
 * stm32f411xx_stm_drivers.h
 *
 *  Created on: Mar 13, 2025
 *      Author: grudz
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

/*
 * This is a Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;			/*!<  >*/
	uint8_t SPI_BusConfig;			/*!<  >*/
	uint8_t SPI_SclkSpeed;			/*!<  >*/
	uint8_t SPI_DFF;		 		/*!<  >*/
	uint8_t SPI_CPOL;				/*!<  >*/
	uint8_t SPI_CPHA;				/*!<  >*/
	uint8_t SPI_SSM;				/*!<  >*/
} SPI_Config_t;

/*
 *  This is a Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; 			/*!< This holds the base addr of the SPIx (x: 0,1,2) peripheral >*/
	SPI_Config_t SPIConfig;
} SPI_Handle_t;



/**************************************************************************************
 *																					  *
 * 										MACROS										  *
 *																					  *
 **************************************************************************************/
/*
 *  @SPI_DeviceMode
 *  Possible device modes for SPI communication
 *  (bit 2 (MSTR) of the CR1 reg)
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0	//default

/*
 *  @ SPI_BusConfig
 *  in CR1 reg: 15th and 14th bit field BIDIMode and BIDIOE (output enable)
 */
#define SPI_BUS_CONFIG_FD				1		//Full Duplex
#define SPI_BUS_CONFIG_HD				2		//Half Duplex
#define	SPI_BUS_CONFIG_SIMPLEX_TXONLY	3		//Simplex Transmit only
#define	SPI_BUS_CONFIG_SIMPLEX_RXONLY	4		//Simplex Recieve only

/*
 *  @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6

/*
 *  @SPI_DFF
 */
#define	SPI_DFF_8BITS	0
#define	SPI_DFF_16BITS	1

/*
 *  @SPI_CPOL
 */
#define SPI_CPOL_HIGH	1	//CK to 1 when idle
#define SPI_CPOL_LOW	0	//CK to 0 when idle

/*
 *  @SPI_CPHA
 */
#define SPI_CPHA_HIGH	1	//The second clock transition is the first data capture edge
#define SPI_CPHA_LOW	0	//The first clock transition is the first data capture edge

/*
 *  @SPI_SSM
 */
#define SPI_SSM_EN		1	//	(software slave menagement enabled)
#define SPI_SSM_DI		0	//	(software slave menagement disabled) - default


/*
 *  SPI related status flags definitions
 */
#define SPI_RXNE_FLAG	( 1 << SPI_SR_RXNE )
#define SPI_TXE_FLAG	( 1 << SPI_SR_TXE )
#define SPI_CHSIDE_FLAG	( 1 << SPI_SR_CHSIDE )
#define SPI_UDR_FLAG	( 1 << SPI_SR_UDR )
#define SPI_CRCERR_FLAG	( 1 << SPI_SR_CRC_ERR )
#define SPI_MODF_FLAG	( 1 << SPI_SR_MODF )
#define SPI_OVR_FLAG	( 1 << SPI_SR_OVR )
#define SPI_BUSY_FLAG	( 1 << SPI_SR_BSY )
#define SPI_FRE_FLAG	( 1 << SPI_SR_FRE )


/**************************************************************************************
 *						APIs supported by this driver
 *		For more information about the APIs check the function definitions
 **************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnORDis); // enable/disable the peripheral clock for given gpio baseaddr

/*
 *  Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 *  Data Send and Recive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 *  IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnORDis);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(void);

/*
 *  Other Peripheral Control APIs
 */



#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
