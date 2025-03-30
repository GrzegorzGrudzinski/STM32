/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Mar 13, 2025
 *      Author: grudz
 *
 *      TODO finish this!!!!!
 */
#include "stm32f411xx.h"
#include "stm32f411xx_spi_driver.h"

/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						- SPI_PeriClockControl
 *
 * @brief					- Peripheral Clock setup for SPI
 *
 * @param[in]				- SPI baseaddr
 * @param[in]				- ENABLE or DISABLE
 *
 * @return					- none
 *
 * @Note					- none
 *
 ******************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnORDis)
{
	if(EnORDis == ENABLE)
	{
		//enable the pin
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLK_EN();
		else if(pSPIx == SPI4)
			SPI4_PCLK_EN();
		else if(pSPIx == SPI5)
			SPI5_PCLK_EN();
	}
	else
	{
		//disable the pin
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLK_DI();
		else if(pSPIx == SPI4)
			SPI4_PCLK_DI();
		else if(pSPIx == SPI5)
			SPI5_PCLK_DI();
	}
}


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						- SPI_DeInit
 *
 * @brief					- Deinitialize the SPI
 *
 * @param[in]				- SPI baseaddr
 *
 * @return					- none
 *
 * @Note					- none
 *
 ******************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
	else if(pSPIx == SPI4)
		SPI4_REG_RESET();
	else if(pSPIx == SPI5)
		SPI5_REG_RESET();
}


/******************************************************************
 * 						FUNCTION DESCRIPTION
 * @fn						-
 *
 * @brief					-
 *
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 ******************************************************************/


