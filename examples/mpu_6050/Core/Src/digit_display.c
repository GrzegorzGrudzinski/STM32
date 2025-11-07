/*
 * digit_display.c
 *
 *  Created on: Jul 23, 2025
 *      Author: grudz
 *
 *      TODO skipping missing fields of the smaller numbers (np. first digit for the num < 100)
 */

#include "stdlib.h"
#include "main.h"
#include "digit_display.h"


#define SEG_A_POS 7
#define SEG_B_POS 6
#define SEG_C_POS 5
#define SEG_D_POS 4
#define SEG_E_POS 3
#define SEG_F_POS 2
#define SEG_G_POS 1
#define SEG_DP_POS 0


const uint8_t digit_patterns[] = {
	/* 0bABCDEFG. */
/*	0b00000000, // NULL */
	0b11111100, // 0
	0b01100000, // 1
	0b11011010, // 2
	0b11110010, // 3
	0b01100110, // 4
	0b10110110, // 5
	0b10111110, // 6
	0b11100000, // 7
	0b11111110, // 8
	0b11110110, // 9
/*	0b00000001, // . - DOT */
};


uint8_t get_the_digit(int16_t number, uint8_t pos) {
	 uint8_t digits[] = {
		 (number / 100) % 10, // hundreds
		 (number / 10) % 10, // tens
		  number % 10 		 // units
	 };

	 return digits[pos];
}

void print_digit(int16_t number, uint8_t pos) {
	uint8_t negative_num = 0x00;
	if (number < 0) {

	  uint16_t temp = number >> 15;     // make a mask of the sign bit
	  number ^= temp;                   // toggle the bits if value is negative
	  number += temp & 1;               // add one if value was negative

	  negative_num = 0b00000001;
	}

	uint8_t pattern = digit_patterns[get_the_digit(number, pos)] | negative_num;

	switch(pos) {
	case 0:
		HAL_GPIO_WritePin(LED_DIGIT_1_GPIO_Port, LED_DIGIT_1_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_DIGIT_2_GPIO_Port, LED_DIGIT_2_Pin, ENABLE);
		HAL_GPIO_WritePin(LED_DIGIT_3_GPIO_Port, LED_DIGIT_3_Pin, ENABLE);
		break;

	case 1:
		HAL_GPIO_WritePin(LED_DIGIT_1_GPIO_Port, LED_DIGIT_1_Pin, ENABLE);
		HAL_GPIO_WritePin(LED_DIGIT_2_GPIO_Port, LED_DIGIT_2_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_DIGIT_3_GPIO_Port, LED_DIGIT_3_Pin, ENABLE);
		break;

	case 2:
		HAL_GPIO_WritePin(LED_DIGIT_1_GPIO_Port, LED_DIGIT_1_Pin, ENABLE);
		HAL_GPIO_WritePin(LED_DIGIT_2_GPIO_Port, LED_DIGIT_2_Pin, ENABLE);
		HAL_GPIO_WritePin(LED_DIGIT_3_GPIO_Port, LED_DIGIT_3_Pin, DISABLE);
		break;
	}

	HAL_GPIO_WritePin(LED_LBL_A_GPIO_Port, LED_LBL_A_Pin, ( pattern & (1 << SEG_A_POS) ));
	HAL_GPIO_WritePin(LED_LBL_B_GPIO_Port, LED_LBL_B_Pin, ( pattern & (1 << SEG_B_POS) ));
	HAL_GPIO_WritePin(LED_LBL_C_GPIO_Port, LED_LBL_C_Pin, ( pattern & (1 << SEG_C_POS) ));
	HAL_GPIO_WritePin(LED_LBL_D_GPIO_Port, LED_LBL_D_Pin, ( pattern & (1 << SEG_D_POS) ));
	HAL_GPIO_WritePin(LED_LBL_E_GPIO_Port, LED_LBL_E_Pin, ( pattern & (1 << SEG_E_POS) ));
	HAL_GPIO_WritePin(LED_LBL_F_GPIO_Port, LED_LBL_F_Pin, ( pattern & (1 << SEG_F_POS) ));
	HAL_GPIO_WritePin(LED_LBL_G_GPIO_Port, LED_LBL_G_Pin, ( pattern & (1 << SEG_G_POS) ));
	HAL_GPIO_WritePin(LED_LBL_DF_GPIO_Port, LED_LBL_DF_Pin, ( pattern & (1 << SEG_DP_POS) ));

}

void print_digits(int16_t number) {
	for (uint8_t i=0; i<3; ++i) {
		print_digit(number, i);
		HAL_Delay(5);
	}
}


void display_test () {
	for (uint8_t i=0; i<3; i++) {
		switch(i) {
		case 0:
			HAL_GPIO_WritePin(LED_DIGIT_1_GPIO_Port, LED_DIGIT_1_Pin, DISABLE);
			HAL_GPIO_WritePin(LED_DIGIT_2_GPIO_Port, LED_DIGIT_2_Pin, ENABLE);
			HAL_GPIO_WritePin(LED_DIGIT_3_GPIO_Port, LED_DIGIT_3_Pin, ENABLE);
			break;

		case 1:
			HAL_GPIO_WritePin(LED_DIGIT_1_GPIO_Port, LED_DIGIT_1_Pin, ENABLE);
			HAL_GPIO_WritePin(LED_DIGIT_2_GPIO_Port, LED_DIGIT_2_Pin, DISABLE);
			HAL_GPIO_WritePin(LED_DIGIT_3_GPIO_Port, LED_DIGIT_3_Pin, ENABLE);
			break;

		case 2:
			HAL_GPIO_WritePin(LED_DIGIT_1_GPIO_Port, LED_DIGIT_1_Pin, ENABLE);
			HAL_GPIO_WritePin(LED_DIGIT_2_GPIO_Port, LED_DIGIT_2_Pin, ENABLE);
			HAL_GPIO_WritePin(LED_DIGIT_3_GPIO_Port, LED_DIGIT_3_Pin, DISABLE);
			break;

		}

		HAL_GPIO_WritePin(LED_LBL_A_GPIO_Port, LED_LBL_A_Pin, ENABLE);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_LBL_B_GPIO_Port, LED_LBL_B_Pin, ENABLE);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_LBL_C_GPIO_Port, LED_LBL_C_Pin, ENABLE);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_LBL_D_GPIO_Port, LED_LBL_D_Pin, ENABLE);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_LBL_E_GPIO_Port, LED_LBL_E_Pin, ENABLE);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_LBL_F_GPIO_Port, LED_LBL_F_Pin, ENABLE);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_LBL_G_GPIO_Port, LED_LBL_G_Pin, ENABLE);
		HAL_Delay(1000);

		HAL_Delay(5);

		HAL_GPIO_WritePin(LED_LBL_A_GPIO_Port, LED_LBL_A_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_LBL_B_GPIO_Port, LED_LBL_B_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_LBL_C_GPIO_Port, LED_LBL_C_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_LBL_D_GPIO_Port, LED_LBL_D_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_LBL_E_GPIO_Port, LED_LBL_E_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_LBL_F_GPIO_Port, LED_LBL_F_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_LBL_G_GPIO_Port, LED_LBL_G_Pin, DISABLE);
	}

	for (int i=0; i<10; i++) {
		uint8_t pattern = digit_patterns[i];

		HAL_GPIO_WritePin(LED_DIGIT_1_GPIO_Port, LED_DIGIT_1_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_DIGIT_2_GPIO_Port, LED_DIGIT_2_Pin, DISABLE);
		HAL_GPIO_WritePin(LED_DIGIT_3_GPIO_Port, LED_DIGIT_3_Pin, DISABLE);

		HAL_GPIO_WritePin(LED_LBL_A_GPIO_Port, LED_LBL_A_Pin, ( pattern & (1 << SEG_A_POS) ));
		HAL_GPIO_WritePin(LED_LBL_B_GPIO_Port, LED_LBL_B_Pin, ( pattern & (1 << SEG_B_POS) ));
		HAL_GPIO_WritePin(LED_LBL_C_GPIO_Port, LED_LBL_C_Pin, ( pattern & (1 << SEG_C_POS) ));
		HAL_GPIO_WritePin(LED_LBL_D_GPIO_Port, LED_LBL_D_Pin, ( pattern & (1 << SEG_D_POS) ));
		HAL_GPIO_WritePin(LED_LBL_E_GPIO_Port, LED_LBL_E_Pin, ( pattern & (1 << SEG_E_POS) ));
		HAL_GPIO_WritePin(LED_LBL_F_GPIO_Port, LED_LBL_F_Pin, ( pattern & (1 << SEG_F_POS) ));
		HAL_GPIO_WritePin(LED_LBL_G_GPIO_Port, LED_LBL_G_Pin, ( pattern & (1 << SEG_G_POS) ));

		HAL_Delay(1000);
	}
}
