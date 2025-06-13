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
 * 				FUNCTION DESCRIPTION
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- Peripheral Clock setup for SPI
 *
 * @param pSPIx		- SPI baseaddr
 * @param EnORDis	- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- Use in master mode
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


/***************************************************************
 * 					FUNCTION DESCRIPTION
 * @fn			- SPI_DeInit
 *
 * @brief		- Deinitialize the SPI
 *
 * @param pSPIx	- SPI baseaddr
 *
 * @return		- none
 *
 * @Note		- none
 *
 ***************************************************************/
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

/***************************************************************
 * 					FUNCTION DESCRIPTION
 * @fn				-
 *
 * @brief			-
 *
 * @param pSPIx		-
 * @param FlagName	-
 *
 * @return			-
 *
 * @Note			- none
 *
 ***************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if(pSPIx->SR.reg & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***************************************************************
 * 					FUNCTION DESCRIPTION
 * @fn				-
 *
 * @brief			-
 *
 * @param pSPIx		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************/
void SPI_Init(SPI_Handle_t *pSPIx) {
	/* Enable the Clock */
	SPI_PeriClockControl(pSPIx->pSPIx, ENABLE);

	/* Disable SPI for the configuration */
	pSPIx->pSPIx->CR1.SPE = DISABLE;

	// Configure control registers
	pSPIx->pSPIx->CR1.reg = 0; // Reset CR1

	pSPIx->pSPIx->CR1.MSTR = pSPIx->SPIConfig.SPI_DeviceMode;
	pSPIx->pSPIx->CR1.BR = pSPIx->SPIConfig.SPI_SclkSpeed;
	pSPIx->pSPIx->CR1.CPOL = pSPIx->SPIConfig.SPI_CPOL;
	pSPIx->pSPIx->CR1.CPHA = pSPIx->SPIConfig.SPI_CPHA;
	pSPIx->pSPIx->CR1.DFF = pSPIx->SPIConfig.SPI_DFF;
	pSPIx->pSPIx->CR1.LSBFIRST = pSPIx->SPIConfig.SPI_LSBFIRST;

	//SSM;
	pSPIx->pSPIx->CR1.SSM = pSPIx->SPIConfig.SPI_SSM ;
	if (pSPIx->SPIConfig.SPI_SSM) {
		pSPIx->pSPIx->CR1.SSI = ENABLE ;
	}
	else {
		pSPIx->pSPIx->CR1.SSI=DISABLE;
		pSPIx->SPIConfig.SPI_DeviceMode ? pSPIx->pSPIx->CR2.SSOE = ENABLE : (pSPIx->pSPIx->CR2.SSOE = DISABLE) ;
	}

	//	 pSPIx->pSPIx->CR2.TXEIE = 1; /* Interrupt mode */

	/* Bus config */
	switch (pSPIx->SPIConfig.SPI_BusConfig) {
	 case SPI_BUS_CONFIG_FD:
		 pSPIx->pSPIx->CR1.BIDIMODE = 0;
		 pSPIx->pSPIx->CR1.RXONLY = 0;
		 break;
	 case SPI_BUS_CONFIG_HD:
		 pSPIx->pSPIx->CR1.BIDIMODE = 1; // 1 clock and 1 bidirectional data wire
		 break;
	 case SPI_BUS_CONFIG_SIMPLEX_RXONLY:
		 pSPIx->pSPIx->CR1.BIDIMODE = 0;
		 pSPIx->pSPIx->CR1.RXONLY = 1;
		 break;
	 }
}

/*************************************************************
 *					FUNCTION DESCRIPTION
 * @fn				- SPI_PeriControl
 *
 * @brief			- enable or disable the SPI
 *
 * @param pSPIx		- SPI baseaddr
 * @param EnORDis	- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************/
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnORDis) {
	pSPIx->CR1.SPE = EnORDis;
}


/******************************************************************
 * 					FUNCTION DESCRIPTION
 * @fn				-
 *
 * @brief			-
 *
 * @param pSPIx		-
 * @param pTxBuff	-
 * @param data_len	-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *	TODO - bsy flag
 *	TODO - interrupt-based mechanism
 ******************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuff, uint32_t data_len) {
/* full duplex / half duplex */
	while (data_len > 0) { //TODO - interrupt-based mechanism /* 1. Wait for the TXE Flag (Tx buff empty) + Interrupt */
//		while (! (pSPIx->SR.TXE)) ;
		while( ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET) );
		if ( pSPIx->CR1.DFF == SPI_DFF_16BITS) {
			/* 16 bit DFF */
			pSPIx->DR.reg = *((uint16_t*) pTxBuff);
			data_len--;
			data_len--;
//			(uint16_t*) pTxBuff++;
			pTxBuff+=2;
		}
		else {
			/* 8 bit DFF */
			pSPIx->DR.reg = *(pTxBuff);
			data_len--;
			pTxBuff++;
		}
	}

//	while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG));
}

/******************************************************************
 * 					FUNCTION DESCRIPTION
 * @fn				-
 *
 * @brief			-
 *
 * @param pSPIx		-
 * @param pRxBuff	-
 * @param data_len	-
 *
 * @return			- none
 *
 * @Note			- none
 *
 ******************************************************************/
void SPI_ReceiveData (SPI_RegDef_t *pSPIx,  uint8_t* pRxBuff, uint32_t data_len) {
	while (data_len > 0) { //TODO - interrupt-based mechanism /* 1. Wait for the TXE Flag (Tx buff empty) + Interrupt */
		/* (RXNE flag) - read */
		while( ( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET) );
		if ( pSPIx->CR1.DFF == SPI_DFF_16BITS) {
			/* 16 bit DFF */
			*((uint16_t*) pRxBuff) = pSPIx->DR.reg;
			data_len--;
			data_len--;
//			(uint16_t*) pRxBuff++;
			pRxBuff+=2;
		}
		else {
			/* 8 bit DFF */
			*pRxBuff = pSPIx->DR.reg;
			data_len--;
			pRxBuff++;
		}
	}
//	while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG));
}


/*
 *
 */
void SPI_SimpleSendData(uint8_t data) {
	while (!(SPI2->SR.reg & (1 << 1))); // Wait TXE
	SPI2->DR.reg = data;
}
