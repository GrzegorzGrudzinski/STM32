/*
 * TIM_register_bits.h
 *
 *  Created on: Mar 5, 2025
 *      Author: grudz
 */

#ifndef INC_REGISTERS_BITS_TIM_REGISTER_BITS_H_
#define INC_REGISTERS_BITS_TIM_REGISTER_BITS_H_

#include "general_definitions.h"

//
union TIMx_CR1_t{
	struct {
		__vo uint32_t CEN 	: 1;	// bit 0 	- counter enable
		__vo uint32_t UDIS 	: 1;	// bit 1 	- update disable
		__vo uint32_t URS	: 1;	// bit 2 	- update request source
		__vo uint32_t OPM 	: 1;	// bit 3 	- one-pulse mode
		__vo uint32_t DIR 	: 1;	// bit 4 	- direction
		__vo uint32_t CMS 	: 2;	// bit 5,6	- center-aligned mode selection
		__vo uint32_t ARPE 	: 1;	// bit 7 	- auto-reload preload enable
		__vo uint32_t CKD 	: 2;	// bit 8,9 	- clock division
		uint32_t RESERVED	: 5;	// bit 10-15
	};
	__vo uint32_t reg;
};

//
union TIMx_CR2_t{
	struct {
		uint32_t RESERVED1	: 3;	// bits 0-2
		__vo uint32_t CCDS 	: 1;	// bit  3 	- update disable
		__vo uint32_t MMS	: 3;	// bit  4 	- update request source
		__vo uint32_t TI1S 	: 1;	// bit  5 	- one-pulse mode
		uint32_t RESERVED2	: 7;	// bits 10-15
	};
	__vo uint32_t reg;
};

//
union TIMx_SMCR_t{
	struct {
		__vo uint32_t SMS 	: 3;	// bit 0-2 	- Slave mode selection
		uint32_t RESERVED 	: 1;	// bit 3
		__vo uint32_t TS	: 3;	// bit 4-6 	- Trigger selection
		__vo uint32_t MSM 	: 1;	// bit 7 	- Master/Slave mode
		__vo uint32_t ETF 	: 4;	// bit 8-11	- External trigger filter
		__vo uint32_t ETPS 	: 2;	// bit 12-13- External trigger prescaler
		__vo uint32_t ECE 	: 1;	// bit 14 	- External clock enable
		__vo uint32_t ETP	: 1;	// bit 15 	- External trigger polarity
	};
	__vo uint32_t reg;
};


//
union TIMx_DIER_t{
	struct {
		__vo uint32_t UIE 	: 1;	// bit 0 	- Update interrupt enable
		__vo uint32_t CC1IE	: 1;	// bit 1 	- Capture/Compare 1 interrupt enable
		__vo uint32_t CC2IE	: 1;	// bit 2 	- Capture/Compare 2 interrupt enable
		__vo uint32_t CC3IE	: 1;	// bit 3	- Capture/Compare 3 interrupt enable
		__vo uint32_t CC4IE	: 1;	// bit 4	- Capture/Compare 4 interrupt enable
		uint32_t  RESERVED1	: 1;	// bit 5
		__vo uint32_t TIE	: 1;	// bit 6	- Trigger interrupt enable
		uint32_t  RESERVED2	: 1;	// bit 7 	-
		__vo uint32_t UDE	: 1;	// bit 8 	- Update DMA request enable
		__vo uint32_t CC1DE	: 1;	// bit 9 	- Capture/Compare 1 DMA request enable
		__vo uint32_t CC2DE	: 1;	// bit 10 	- Capture/Compare 2 DMA request enable
		__vo uint32_t CC3DE	: 1;	// bit 11 	- Capture/Compare 3 DMA request enable
		__vo uint32_t CC4DE	: 1;	// bit 12 	- Capture/Compare 4 DMA request enable
		uint32_t RESERVED3	: 1;	// bit 13
		__vo uint32_t TDE	: 1;	// bit 14 	-  Trigger DMA request enable
		uint32_t RESERVED4	: 1;	// bit 15
	};
	__vo uint32_t reg;
};

//
union TIMx_SR_t{
	struct {
		__vo uint32_t UIF 	: 1;	// bit 0 	- Update interrupt flag
		__vo uint32_t CC1IF	: 1;	// bit 1 	- Capture/Compare 1 interrupt flag
		__vo uint32_t CC2IF	: 1;	// bit 2 	- Capture/Compare 2 interrupt flag
		__vo uint32_t CC3IF	: 1;	// bit 3 	- Capture/Compare 3 interrupt flag
		__vo uint32_t CC4IF	: 1;	// bit 4 	- Capture/Compare 4 interrupt flag
		uint32_t RESERVED1	: 1;	// bit 5
		__vo uint32_t TIF 	: 1;	// bit 6	- Trigger interrupt flag
		uint32_t RESERVED2	: 2;	// bits 7,8
		__vo uint32_t CC1OF : 1;	// bit 9	- Capture/Compare 1 overcapture flag
		__vo uint32_t CC2OF	: 1;	// bit 10	- Capture/Compare 2 overcapture flag
		__vo uint32_t CC3OF	: 1;	// bit 11	- Capture/Compare 3 overcapture flag
		__vo uint32_t CC4OF	: 1;	// bit 12	- Capture/Compare 4 overcapture flag
		uint32_t RESERVED3	: 3;	// bits 13-15
	};
	__vo uint32_t reg;
};

//
union TIMx_EGR_t{
	struct {
		__vo uint32_t UG 	: 1;	// bit 0 	- Update generation
		__vo uint32_t CC1G	: 1;	// bit 1 	- Capture/compare 1 generation
		__vo uint32_t CC2G 	: 1;	// bit 2 	- Capture/compare 2 generation
		__vo uint32_t CC3G 	: 1;	// bit 3	- Capture/compare 3 generation
		__vo uint32_t CC4G 	: 1;	// bit 4	- Capture/compare 4 generation
		uint32_t RESERVED1	: 1;	// bit 5
		__vo uint32_t TG	: 1;	// bit 6 	- Trigger generation
		uint32_t RESERVED2	: 9;	// bits 7-15
	};
	__vo uint32_t reg;
};

//
union TIMx_CCMR1_t{
	struct {
		__vo uint32_t CC1S 	: 2;	// bit 0,1 	- Capture/Compare 1 selection
		__vo uint32_t OC1FE	: 1;	// bit 2 	- Output compare 1 fast enable
		__vo uint32_t OC1PE	: 1;	// bit 3 	- Output compare 1 preload enable
		__vo uint32_t OC1M 	: 3;	// bits 4-6	- Output compare 1 mode
		__vo uint32_t OC1CE : 1;	// bit 7	- Output compare 1 clear enable
		__vo uint32_t CC2S 	: 2;	// bits 8,9 - Capture/Compare 2 selection
		__vo uint32_t OC2FE	: 1;	// bit 10 	- Output compare 2 fast enable
		__vo uint32_t OC2PE	: 1;	// bit 11 	- Output compare 2 preload enable
		__vo uint32_t OC2M	: 3;	// bit 12-14- Output compare 2 preload enable
		__vo uint32_t OC2CE	: 1;	// bit 15	- Output compare 2 clear enable
	} Output;
	struct {
		__vo uint32_t CC1S 	: 2;	// bits 0,1 	- Capture/Compare 1 selection
		__vo uint32_t IC1PSC: 2;	// bits 2,3 	- Input capture 1 prescaler
		__vo uint32_t IC1F	: 3;	// bits 4-7 	- Input capture 1 filteR
		__vo uint32_t CC2S 	: 2;	// bits 8-9		- Capture/Compare 2 selection
		__vo uint32_t IC2PSC: 2;	// bits 10-11	- Input capture 2 prescaler
		__vo uint32_t IC2F 	: 4;	// bits 12-15	- Input capture 2 filter
	} Input;
	__vo uint32_t reg;
};


union TIMx_CCMR2_t{
	struct {
		__vo uint32_t CC3S 	: 2;	// bit 0,1 	- Capture/Compare 1 selection
		__vo uint32_t OC3FE	: 1;	// bit 2 	- Output compare 1 fast enable
		__vo uint32_t OC3PE	: 1;	// bit 3 	- Output compare 1 preload enable
		__vo uint32_t OC3M 	: 3;	// bits 4-6	- Output compare 1 mode
		__vo uint32_t OC3CE : 1;	// bit 7	- Output compare 1 clear enable
		__vo uint32_t CC4S 	: 2;	// bits 8,9 - Capture/Compare 2 selection
		__vo uint32_t OC4FE	: 1;	// bit 10 	- Output compare 2 fast enable
		__vo uint32_t OC4PE	: 1;	// bit 11 	- Output compare 2 preload enable
		__vo uint32_t OC4M	: 3;	// bit 12-14- Output compare 2 preload enable
		__vo uint32_t OC4CE	: 1;	// bit 15	- Output compare 2 clear enable
	} Output;
	struct {
		__vo uint32_t CC3S 	: 2;	// bits 0,1 	- Capture/Compare 3 selection
		__vo uint32_t IC3PSC: 2;	// bits 2,3 	- Input capture 3 prescaler
		__vo uint32_t IC3F	: 3;	// bits 4-7 	- Input capture 3 filteR
		__vo uint32_t CC4S 	: 2;	// bits 8-9		- Capture/Compare 4 selection
		__vo uint32_t IC4PSC: 2;	// bits 10-11	- Input capture 4 prescaler
		__vo uint32_t IC4F 	: 4;	// bits 12-15	- Input capture 4 filter
	} Input;
	__vo uint32_t reg;
};

//
union TIMx_CCER_t{
	struct {
		__vo uint32_t CC1E 	: 1;	// bit 0 	- Capture/Compare 1 output enable
		__vo uint32_t CC1P	: 1;	// bit 1 	- Capture/Compare 1 output Polarity.
		uint32_t RESERVED1	: 1;	// bit 2
		__vo uint32_t CC1NP	: 1;	// bit 3 	- Capture/Compare 1 output Polarity.
		__vo uint32_t CC2E 	: 1;	// bit 4	- Capture/Compare 2 output enable
		__vo uint32_t CC2P 	: 1;	// bit 5	- Capture/Compare 2 output Polarity.
		uint32_t RESERVED2	: 1;	// bit 6
		__vo uint32_t CC2NP	: 1;	// bit 7	- Capture/Compare 2 output Polarity.
		__vo uint32_t CC3E 	: 1;	// bit 8 	- Capture/Compare 3 output enable
		__vo uint32_t CC3P	: 1;	// bit 9 	- Capture/Compare 3 output Polarity.
		uint32_t RESERVED3	: 1;	// bit 10
		__vo uint32_t CC3NP	: 1;	// bit 11	- Capture/Compare 3 output Polarity.
		__vo uint32_t CC4E 	: 1;	// bit 12 	- Capture/Compare 3 output enable
		__vo uint32_t CC4P	: 1;	// bit 13 	- Capture/Compare 4 output Polarity.
		uint32_t RESERVED4	: 1;	// bit 14
		__vo uint32_t CC4NP	: 1;	// bit 15	- Capture/Compare 4 output Polarity.
	};
	__vo uint32_t reg;
};

//
union TIMx_CNT_t{
	struct {
		__vo uint32_t CNT : 16;	// bits 0-15  -  Counter value
		__vo uint32_t CNT_High: 16;	// bit  16-31 -  High counter value (on TIM2 and TIM5)
	};
	__vo uint32_t reg;
};

//
union  TIMx_PSC_t{
	struct {
		__vo uint32_t PSC : 16;	// bits 0-15  - Prescaler value
	};
	__vo uint32_t reg;
};

//
union TIMx_ARR_t{
	struct {
		__vo uint32_t ARR : 16;	// bits 0-15  - Auto-reload value
	};
	__vo uint32_t reg;
};


/*
 If channel CC1 is configured as output:
 CCR1 is the value to be loaded in the actual capture/compare 1 register (preload value).
 It is loaded permanently if the preload feature is not selected in the TIMx_CCMR1 register
(bit OC1PE). Else the preload value is copied in the active capture/compare 1 register when
an update event occurs.
 The active capture/compare register contains the value to be compared to the counter
TIMx_CNT and signaled on OC1 output.
 If channel CC1is configured as input:
 CCR1 is the counter value transferred by the last input capture 1 event (IC1). The
TIMx_CCR1 register is read-only and cannot be programmed.
 */
union TIMx_CCR_t{
	struct {
		__vo uint32_t CCR_LOW 	: 16;	// bits 0-15 	-  Low Capture/Compare 1 value
		__vo uint32_t CCR_HIGH	: 16;	// bits 16-31 	-  High Capture/Compare 1 value (on TIM2 and TIM5)
	};
	__vo uint32_t reg;
};

union TIMx_DCR_t{
	struct {
		__vo uint32_t DBA 	: 5;	// bits 0-4   - DMA base address
		uint32_t RESERVED1	: 3;	// bits 5-7
		__vo uint32_t DBL 	: 5;	// bits 8-12  - DMA burst length
		uint32_t RESERVED2	: 3;	// bits 13-15
	};
	__vo uint32_t reg;
};

//
union TIMx_DMAR_t{
	struct {
		__vo uint32_t DMAB 	: 16;	// bits 0-15 - DMA register for burst accesses
	};
	__vo uint32_t reg;
};

//
union TIMx_OR_t{
	struct {
		uint32_t RESERVED1 : 10;	// bits 0-9
		__vo uint32_t ITR1_RMP: 2;	// bits 10-11 - Internal trigger 1 remap
		uint32_t RESERVED2 : 4;		// bits 12-15
	}TIM2;
	struct {
		uint32_t RESERVED1 : 6;		// bits 0-5
		__vo uint32_t TI4_RMP: 2;	// bits 6-7 - Timer Input 4 remap
		uint32_t RESERVED2 : 8;		// bits 8-15
	}TIM5;
	__vo uint32_t reg;
};


#endif /* INC_REGISTERS_BITS_TIM_REGISTER_BITS_H_ */
