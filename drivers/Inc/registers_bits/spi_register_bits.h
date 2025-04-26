/*
 * spi_register_bits.h
 *
 *  Created on: Mar 13, 2025
 *      Author: grudz
 */

#ifndef INC_REGISTERS_BITS_SPI_REGISTER_BITS_H_
#define INC_REGISTERS_BITS_SPI_REGISTER_BITS_H_

#include "general_definitions.h"

union SPI_CR1_t
{
	struct {
		__vo uint32_t CPHA	: 1;	/*!< bit 0 - Clock phase  		 >*/
		__vo uint32_t CPOL	: 1;	/*!< bit 1 - Clock polarity  	 >*/
		__vo uint32_t MSTR	: 1;	/*!< bit 2 - Master selection 	 >*/
		__vo uint32_t BR	: 3;	/*!< bits 3-5 -Baud rate controL 	 >*/
		__vo uint32_t SPE	: 1;	/*!< bit 6 - SPI Enable >*/
		__vo uint32_t LSBFIRST: 1;	/*!< bit 7 - frame format >*/
		__vo uint32_t SSI	: 1;	/*!< bit 8 - Internal slave select >*/
		__vo uint32_t SSM	: 1;	/*!< bit 9 - Software slave management >*/
		__vo uint32_t RXONLY: 1;	/*!< bit 10- Receive only  >*/
		__vo uint32_t DFF	: 1;	/*!< bit 11- Data frame format >*/
		__vo uint32_t CRCNEXT: 1;	/*!< bit 12- CRC transfer next >*/
		__vo uint32_t CRCEN	: 1;	/*!< bit 13- Hardware CRC calculation enable>*/
		__vo uint32_t BIDIOE: 1;	/*!< bit 14- Output enable in bidirectional mode >*/
		__vo uint32_t BIDIMODE: 1;	/*!< bit 15- Bidirectional data mode enable >*/
		uint32_t RESERVED	: 16;	/*!< bits 16-31 >*/
	};
	__vo uint32_t reg;
};


union SPI_CR2_t
{
	struct {
		__vo uint32_t RXDMAEN	: 1;	/*!< bit 0 -  Rx buffer DMA enablE >*/
		__vo uint32_t TXDMAEN	: 1;	/*!< bit 1 -  Tx buffer DMA enable >*/
		__vo uint32_t SSOE	: 1;	/*!< bit 2 - SS output enable - This bit is not used in I2S mode. >*/
		uint32_t RESERVED0	: 1;	/*!< bit 3>*/
		__vo uint32_t FRF	: 1;	/*!< bit 4 -  Frame format - This bit is not used in I2S mode. >*/
		__vo uint32_t ERRIE	: 1;	/*!< bit 5 - Error interrupt enable >*/
		__vo uint32_t RXNEIE: 1;	/*!< bit 6 - RX buffer not empty interrupt enable >*/
		__vo uint32_t TXEIE	: 1;	/*!< bit 7 - Tx buffer empty interrupt enable >*/
		uint32_t RESERVED1	: 8;	/*!< bits 8-15 >*/
		uint32_t RESERVED2	: 16;	/*!< bits 16-31 >*/
	};
	__vo uint32_t reg;
};

union SPI_SR_t
{
	struct {
		__vo uint32_t RXNE	: 1;	/*!< bit 0 - Receive buffer not emptY >*/
		__vo uint32_t TXE	: 1;	/*!< bit 1 - Transmit buffer emptY >*/
		__vo uint32_t CHSIDE: 1;	/*!< bit 2 - Channel side -  This bit is not used for SPI mode and is meaningless in PCM mode >*/
		__vo uint32_t UDR	: 1;	/*!< bit 3 - Underrun flag - This bit is not used in SPI mode >*/
		__vo uint32_t CRCERR	: 1;	/*!< bit 4 - CRC error flag - This bit is not used in I2S mode >*/
		__vo uint32_t MODF 	: 1;	/*!< bit 5 - Mode fault - This bit is not used in I2S mode >*/
		__vo uint32_t OVR	: 1;	/*!< bit 6 - Overrun flag >*/
		__vo uint32_t BSY	: 1;	/*!< bit 7 - Busy flag  >*/
		__vo uint32_t FRE	: 1;	/*!< bit 8 - Frame format error >*/
		uint32_t RESERVED	: 7;	/*!< bits 9-15 >*/
		uint32_t RESERVED1	: 16;	/*!< bits 16-31 >*/

	};
	__vo uint32_t reg;
};

union SPI_DR_t
{
	struct {
		__vo uint32_t DR	: 16;	/*!< bits 0-15 - Data register >*/
		uint32_t RESERVED1	: 16;	/*!< bits 16-31 >*/

	};
	__vo uint32_t reg;
};


union SPI_CRCPR_t
{
	struct {
		__vo uint32_t CRCPOLY : 16;	/*!< bitS 0-15 - CRC polynomial register >*/
		uint32_t RESERVED1	  : 16;	/*!< bits 16-31 >*/

	};
	__vo uint32_t reg;
};


union SPI_RXCRCR_t
{
	struct {
		__vo uint32_t RXCRC : 16;	/*!< bitS 0-15 - Rx CRC registeR >*/
		uint32_t RESERVED1	: 16;	/*!< bits 16-31 >*/

	};
	__vo uint32_t reg;
};


union SPI_TXCRCR_t
{
	struct {
		__vo uint32_t TXCRC : 16;	/*!< bitS 0-15 - Tx CRC registeR >*/
		uint32_t RESERVED1	: 16;	/*!< bits 16-31 >*/

	};
	__vo uint32_t reg;
};


union SPI_I2SCFGR_t
{
	struct {
		__vo uint32_t CHLEN	 : 1;	/*!< bit 0 - Channel length (number of bits per audio channel >*/
		__vo uint32_t DATLEN : 2;	/*!< bit 1-2 - Data length to be transferred >*/
		__vo uint32_t CKPOL	 : 1;	/*!< bit 3 - Steady state clock polarity >*/
		__vo uint32_t I2SSTD : 2;	/*!< bit 4-5 -  I2S standard selectio >*/
		uint32_t RESERVED	 : 1;	/*!< bit 6 >*/
		__vo uint32_t PCMSYNC: 1;	/*!< bit 7 - PCM frame synchronization >*/
		__vo uint32_t I2SCFG : 2;	/*!< bit 8,9 - I2S configuration mode >*/
		__vo uint32_t I2SE	 : 1;	/*!< bit 10 - I2S Enable >*/
		__vo uint32_t I2SMOD : 1;	/*!< bit 11 - I2S mode selection >*/
		uint32_t RESERVED0	 : 4;	/*!< bits 12-15 >*/
		uint32_t RESERVED1	 : 16;	/*!< bits 16-31 >*/

	};
	__vo uint32_t reg;
};

union SPI_I2SPR_t
{
	struct {
		__vo uint32_t I2SDIV : 8;	/*!< bits 0-7 - I2S Linear prescaler >*/
		__vo uint32_t ODD	 : 1;	/*!< bit 8 - Odd factor for the prescaler >*/
		__vo uint32_t MCKOE	 : 1;	/*!< bit 9 - Master clock output enable >*/
		uint32_t RESERVED	: 6;	/*!< bits 10-15 >*/
		uint32_t RESERVED1	: 16;	/*!< bits 16-31 >*/

	};
	__vo uint32_t reg;
};


#endif /* INC_REGISTERS_BITS_SPI_REGISTER_BITS_H_ */
