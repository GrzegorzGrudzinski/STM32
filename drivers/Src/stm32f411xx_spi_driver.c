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
	/* Disable SPI for the configuration */
	pSPIx->pSPIx->CR1.SPE = DISABLE;
	/* Enable the Clock */
	SPI_PeriClockControl(pSPIx->pSPIx, ENABLE);

	pSPIx->pSPIx->CR1.BR = pSPIx->SPIConfig.SPI_SclkSpeed;
	pSPIx->pSPIx->CR1.CPOL = pSPIx->SPIConfig.SPI_CPOL;
	pSPIx->pSPIx->CR1.CPHA = pSPIx->SPIConfig.SPI_CPHA;
	pSPIx->pSPIx->CR1.DFF = pSPIx->SPIConfig.SPI_DFF; //ERR?? Num 3 in procedure (here: 4)
	pSPIx->pSPIx->CR1.LSBFIRST = pSPIx->SPIConfig.SPI_LSBFIRST; // pSPIx->SPIConfig.SPI_;

	//SSM;
	pSPIx->pSPIx->CR1.SSM = pSPIx->SPIConfig.SPI_SSM ;
	pSPIx->SPIConfig.SPI_SSM ? pSPIx->pSPIx->CR1.SSI = ENABLE : (pSPIx->pSPIx->CR1.SSI=DISABLE);
	pSPIx->SPIConfig.SPI_DeviceMode ? pSPIx->pSPIx->CR2.SSOE = ENABLE : (pSPIx->pSPIx->CR2.SSOE = DISABLE) ;
	// TODO - add nss_req macro and conf space in conf struct, finish this!!
//	char NSS_REQ = 0;
//	if (NSS_REQ) {
//		 pSPIx->pSPIx->CR1.SSM = pSPIx->SPIConfig.SPI_SSM ;
//		/* No multi-master mode (NSS pulled internally to high */
//		 pSPIx->pSPIx->CR1.SSI = 0;
//	 }
//	 else {
//		 pSPIx->pSPIx->CR2.SSOE = 0;
//	 }

	 pSPIx->pSPIx->CR1.MSTR = 1;// pSPIx->SPIConfig.SPI_DeviceMode; // step 7 in conf procedure !!!
/*
//	int TI_MODE = 0; //TODO - add macro and conf space in conf struct
//	if ( !TI_MODE ) {



	//		 // TODO - add nss_req macro and conf space in conf struct, finish this!!
	//		 if (NSS_REQ) {
	//			 pSPIx->pSPIx->CR1.SSM = pSPIx->SPIConfig.SPI_SSM ;
	//			 pSPIx->pSPIx->CR1.SSI = 0;
	//		 }
	//		 else {
	//			 pSPIx->pSPIx->CR2.SSOE = 0;
	//		 }
//	}
	//	 else { //TODO - TI Mode
	//		 pSPIx->pSPIx->CR2.FRF = TI_MODE;
	//	 }*/

	//	 pSPIx->pSPIx->CR2.TXEIE = 1; /* Interrupt mode */

	/* Bus config */
	switch (pSPIx->SPIConfig.SPI_BusConfig) {
	 case SPI_BUS_CONFIG_FD:
		 //full duplex
		 pSPIx->pSPIx->CR1.BIDIMODE = 0;
		 pSPIx->pSPIx->CR1.RXONLY = 0;
		 break;
	 case SPI_BUS_CONFIG_HD:
		 //half duplex
		 pSPIx->pSPIx->CR1.BIDIMODE = 1; // 1 clock and 1 bidirectional data wire
	//			 pSPIx->pSPIx->CR1.BIDIOE = 1;
		 /*
		  The transfer direction (Input/Output) is selected by the BIDIOE bit in the
		  SPI_CR1 register. When this bit is 1, the data line is output otherwise it
		  is input.
		 */
		 break;
	 case SPI_BUS_CONFIG_SIMPLEX_RXONLY:
		//simplex
		 pSPIx->pSPIx->CR1.BIDIMODE = 0;
		 pSPIx->pSPIx->CR1.RXONLY = 1;
		 break;
	 //		 case SPI_BUS_CONFIG_SIMPLEX_TXONLY:
	 //			 //simplex
	 //			 pSPIx->pSPIx->CR1.BIDIMODE = 1;
	 //			 pSPIx->pSPIx->CR1.BIDIOE = 0;
	 //			 break;
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
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t* pTxBuff, uint32_t data_len) {
/* full duplex / half duplex */
	while (data_len > 0) { //TODO - interrupt-based mechanism /* 1. Wait for the TXE Flag (Tx buff empty) + Interrupt */
		while (!pSPIx->SR.TXE) ;
		if ( pSPIx->CR1.DFF == SPI_DFF_16BITS) {
			/* 16 bit DFF */
			pSPIx->DR.reg = *((uint16_t*) pTxBuff);
			data_len--;
			data_len--;
			(uint16_t*) pTxBuff++;
		}
		else {
			/* 8 bit DFF */
			pSPIx->DR.reg = *((uint8_t*)pTxBuff);
			data_len--;
			(uint8_t*) pTxBuff++;
		}

		/* (RXNE flag) - read */
//		if (pSPIx->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
//			while ( ! pSPIx->pSPIx->SR.RXNE ) ; //TODO - interrupt-based mechanism
//		}
	}
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
	while (data_len >= 0) { //TODO - interrupt-based mechanism /* 1. Wait for the TXE Flag (Tx buff empty) + Interrupt */
		while ( ! pSPIx->SR.RXNE ) ;
		if ( pSPIx->CR1.DFF == SPI_DFF_16BITS) {
			/* 16 bit DFF */
			*((uint16_t*) pRxBuff) = pSPIx->DR.reg;
			data_len--;
			data_len--;
			(uint16_t*) pRxBuff++;
		}
		else {
			/* 8 bit DFF */
			*pRxBuff = pSPIx->DR.reg;
			data_len--;
			(uint8_t*) pRxBuff++;
		}

	}
}
