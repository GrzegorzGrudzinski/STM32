/*
 * adc_register_bits.h
 *
 *  Created on: Mar 5, 2025
 *      Author: grudz
 */

#ifndef INC_REGISTERS_BITS_ADC_REGISTER_BITS_H_
#define INC_REGISTERS_BITS_ADC_REGISTER_BITS_H_

#include "general_definitions.h"

union ADC_SR_t
{
	struct {
		__vo uint32_t AWD	: 1;	/*!< bit 0 -	Analog watchdog flag >*/
		__vo uint32_t EOC	: 1;	/*!< bit 1 -	Regular channel end of conversion >*/
		__vo uint32_t JEOC	: 1;	/*!< bit 2 -	Injected channel end of conversion >*/
		__vo uint32_t JSTRT	: 1;	/*!< bit 3 -	Injected channel start flag >*/
		__vo uint32_t STRT	: 1;	/*!< bit 4 -	Regular channel start flag >*/
		__vo uint32_t OVR	: 1;	/*!< bit 4 -	Overrun >*/
		uint32_t RESERVED	: 26;	/*!< bits 6-31 >*/
	};
	__vo uint32_t reg;
};

//
union ADC_CR1_t {
	struct {
		__vo uint32_t AWDCH		: 5;	/*!< bits 0-4 - Analog watchdog channel select bits >*/
		__vo uint32_t EOCIE 	: 1;	/*!< bit  5 - Interrupt enable for EOC >*/
		__vo uint32_t AWDIE 	: 1;	/*!< bit  6 - Analog watchdog interrupt enable >*/
		__vo uint32_t JEOCIE	: 1;	/*!< bit  7 - Interrupt enable for injected channels >*/
		__vo uint32_t SCAN		: 1;	/*!< bit  8 - Scan mode >*/
		__vo uint32_t AWDSGL	: 1;	/*!< bit  9 - Enable the watchdog on a single channel in scan mode >*/
		__vo uint32_t JAUTO		: 1;	/*!< bit  10 - Automatic injected group conversion >*/
		__vo uint32_t DISCEN	: 1;	/*!< bit  11 - Discontinuous mode on regular channels >*/
		__vo uint32_t JDISCEN	: 1;	/*!< bit  12 - Discontinuous mode on injected channels >*/
		__vo uint32_t DISCNUM	: 3;	/*!< bit  13-15 - Discontinuous mode channel count >*/
		uint32_t RESERVED1		: 6;
		__vo uint32_t JAWDEN	: 1;	/*!< bit  22 - Analog watchdog enable on injected channels >*/
		__vo uint32_t AWDEN		: 1;	/*!< bit  23 - Analog watchdog enable on regular channels >*/
		__vo uint32_t RES		: 2;	/*!< bit  24-25 - Resolution>*/
		__vo uint32_t OVRIE		: 1;	/*!< bit  26 - Overrun interrupt enable>*/
		uint32_t RESERVED2		: 5;
	};
	__vo uint32_t reg;
};


//
union ADC_CR2_t {
	struct {
		__vo uint32_t ADON		: 1;	/*!< bit 0 -  A/D Converter ON / OFF >*/
		__vo uint32_t CONT		: 1;	/*!< bit 1 -  Continuous conversion >*/
		uint32_t RESERVED1		: 6;	//bits 2-7
		__vo uint32_t DMA		: 1;	/*!< bit 8 - Direct memory access mode (for single ADC mode) >*/
		__vo uint32_t DDS		: 1;	/*!< bit 9 - DMA disable selection (for single ADC mode) >*/
		__vo uint32_t EOCS		: 1;	/*!< bit 10- End of conversion selection >*/
		__vo uint32_t ALIGN		: 1;	/*!< bit 11- Data alignment >*/
		uint32_t RESERVED2		: 4;	//bits 12-15
		__vo uint32_t JEXTSEL	: 4;	/*!< bits 16-19- External event select for injected group >*/
		__vo uint32_t JEXTEN	: 2;	/*!< bits 20-21- External trigger enable for injected channels >*/
		__vo uint32_t JSWSTART	: 1;	/*!< bit 22 - Start conversion of injected channels >*/
		uint32_t RESERVED3		: 1;	//bit 23
		__vo uint32_t EXTSEL	: 4;	/*!< bits 24-27- External event select for regular group >*/
		__vo uint32_t EXTEN		: 2;	/*!< bits 28-29- External trigger enable for regular channels >*/
		__vo uint32_t SWSTART	: 1;	/*!< bit 30 - Start conversion of regular channels >*/
		uint32_t RESERVED4		: 1;	//bit 31
	};
	__vo uint32_t reg;
};

//
union ADC_SMPR1_t {
	struct {
		//Channel x sampling time selection
		__vo uint32_t SMP10		: 3;	/*!< bit 0-2  >*/
		__vo uint32_t SMP11		: 3;	/*!< bit 3-5  >*/
		__vo uint32_t SMP12		: 3;	/*!< bit 6-8  >*/
		__vo uint32_t SMP13		: 3;	/*!< bit 9-11  >*/
		__vo uint32_t SMP14		: 3;	/*!< bit 12-14  >*/
		__vo uint32_t SMP15		: 3;	/*!< bit 15-17  >*/
		__vo uint32_t SMP16		: 3;	/*!< bit 18-20  >*/
		__vo uint32_t SMP17		: 3;	/*!< bit 21-23  >*/
		__vo uint32_t SMP18		: 3;	/*!< bit 24-26  >*/
		uint32_t RESERVED		: 5;	//bits 27-31
	};
	__vo uint32_t reg;
};

//
union ADC_SMPR2_t {
	struct {
		//Channel x sampling time selection
		__vo uint32_t SMP0		: 3;	/*!< bit 0-2  >*/
		__vo uint32_t SMP1		: 3;	/*!< bit 3-5  >*/
		__vo uint32_t SMP2		: 3;	/*!< bit 6-8  >*/
		__vo uint32_t SMP3		: 3;	/*!< bit 9-11  >*/
		__vo uint32_t SMP4		: 3;	/*!< bit 12-14  >*/
		__vo uint32_t SMP5		: 3;	/*!< bit 15-17  >*/
		__vo uint32_t SMP6		: 3;	/*!< bit 18-20  >*/
		__vo uint32_t SMP7		: 3;	/*!< bit 21-23  >*/
		__vo uint32_t SMP8		: 3;	/*!< bit 24-26  >*/
		__vo uint32_t SMP9		: 3;	/*!< bit 27-29  >*/
		uint32_t RESERVED		: 2;	//bits 30-31
	};
	__vo uint32_t reg;
};

//
union ADC_JOFRx_t {
	struct {
		__vo uint32_t JOFFSET	: 12;	/*!< bit 0-11 - Data offset for injected channel x >*/
		uint32_t RESERVED		: 20;	//bits 12-31
	};
	__vo uint32_t reg;
};

//
union ADC_HTR_t {
	struct {
		__vo uint32_t HT	: 12;	/*!< bit 0-11 - Analog watchdog higher threshold >*/
		uint32_t RESERVED	: 20;	//bits 12-31
	};
	__vo uint32_t reg;
};

//
union ADC_LTR_t {
	struct {
		__vo uint32_t LT	: 12;	/*!< bit 0-11 - Analog watchdog lower threshold >*/
		uint32_t RESERVED	: 20;	//bits 12-31
	};
	__vo uint32_t reg;
};

//
union ADC_SQR1_t {
	struct {
		//Channel x sampling time selection
		__vo uint32_t SQ13	: 5;	/*!< bits 0-4 - x conversion in regular sequence  >*/
		__vo uint32_t SQ14	: 5;	/*!< bits 5-9 - x conversion in regular sequence >*/
		__vo uint32_t SQ15	: 5;	/*!< bits 10-14- x conversion in regular sequence >*/
		__vo uint32_t SQ16	: 5;	/*!< bits 15-19 - x conversion in regular sequence >*/
		__vo uint32_t L		: 4;	/*!< bits 20-23 - Regular channel sequence length >*/
		uint32_t RESERVED	: 8;	//bits 30-31
	};
	__vo uint32_t reg;
};

//
union ADC_SQR2_t {
	struct {
		//Channel x sampling time selection
		__vo uint32_t SQ7	: 5;	/*!< bits 0-4 - x conversion in regular sequence  >*/
		__vo uint32_t SQ8	: 5;	/*!< bits 5-9 - x conversion in regular sequence >*/
		__vo uint32_t SQ9	: 5;	/*!< bits 10-14- x conversion in regular sequence >*/
		__vo uint32_t SQ10	: 5;	/*!< bits 15-19 - x conversion in regular sequence >*/
		__vo uint32_t SQ11	: 5;	/*!< bits 20-24 - x conversion in regular sequence >*/
		__vo uint32_t SQ12	: 5;	/*!< bits 25-29 - x conversion in regular sequence >*/
		uint32_t RESERVED	: 2;	//bits 30-31
	};
	__vo uint32_t reg;
};

//
union ADC_SQR3_t {
	struct {
		//Channel x sampling time selection
		__vo uint32_t SQ1	: 5;	/*!< bits 0-4 - x conversion in regular sequence  >*/
		__vo uint32_t SQ2	: 5;	/*!< bits 5-9 - x conversion in regular sequence >*/
		__vo uint32_t SQ3	: 5;	/*!< bits 10-14- x conversion in regular sequence >*/
		__vo uint32_t SQ4	: 5;	/*!< bits 15-19 - x conversion in regular sequence >*/
		__vo uint32_t SQ5	: 5;	/*!< bits 20-24 - x conversion in regular sequence >*/
		__vo uint32_t SQ6	: 5;	/*!< bits 25-29 - x conversion in regular sequence >*/
		uint32_t RESERVED	: 2;	//bits 30-31
	};
	__vo uint32_t reg;
};

//
union ADC_JSQR_t {
	struct {
		//Channel x sampling time selection
		__vo uint32_t JSQ1	: 5;	/*!< bits 0-4 - x conversion in injected sequence  >*/
		__vo uint32_t JSQ2	: 5;	/*!< bits 5-9 - x conversion in injected sequence >*/
		__vo uint32_t JSQ3	: 5;	/*!< bits 10-14-x conversion in injected sequence >*/
		__vo uint32_t JSQ4	: 5;	/*!< bits 15-19-x conversion in injected sequence >*/
		__vo uint32_t JL	: 2;	/*!< bits 20-21 - Injected sequence length >*/
		uint32_t RESERVED	: 10;	//bits 22-31
	};
	__vo uint32_t reg;
};

//
union ADC_JDR_t {
	struct {
		__vo uint32_t JDATA	: 16;	/*!< bits 0-15 - Injected data (r. only) >*/
		uint32_t RESERVED	: 16;	//bits 16-31
	};
	__vo uint32_t reg;
};

//
union ADC_DR_t {
	struct {
		__vo uint32_t DATA	: 16;	/*!< bits 0-15 - Regular data (r. only) >*/
		uint32_t RESERVED	: 16;	//bits 16-31
	};
	__vo uint32_t reg;
};

/*********************************
 *
 * ADC common control register
 *
 *********************************/

//
union ADC_CCR_t {
	struct {
		uint32_t RESERVED1		: 16;	//bits 0-15
		__vo uint32_t ADCPRE	: 2;	/*!< bits 16-17 - ADC prescaler >*/
		uint32_t RESERVED2		: 4;	//bits 18-21
		__vo uint32_t VBATE		: 1;	/*!< bit 22 -  Vbat enable >*/
		__vo uint32_t TSVREFE	: 1;	/*!< bit 23 - Temperature sensor and Vrefint enable >*/
		uint32_t RESERVED3		: 8;	//bits 24-31
	};
	__vo uint32_t reg;
};




#endif /* INC_REGISTERS_BITS_ADC_REGISTER_BITS_H_ */
