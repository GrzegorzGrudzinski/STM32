/*
 *  Created on: Mar 2, 2025
 *      Author: grudz
 */

#include <stdlib.h>
#include <string.h>
#include "stm32f411xx.h"

#include "gyro_I3G4250D.h"


/*************************************************
		GENERAL FUNCTIONS AND MACROS
 *************************************************/
#define DUMMY_BYTE	((uint8_t) 0xff)

#define REVERSE_BIT_POS(X)	reverse_bit_pos (X);

uint8_t reverse_bit_pos (uint8_t x) {
	return 7-x;
}

void Reverse_bits (uint8_t* value) {
	uint8_t rev_temp = 0;
	for (int i=sizeof(*value)*7; i>=0; i--) {
		rev_temp |= ((*value >> i) & 0x1) << (7-i);
	}

	*value = rev_temp;
}


/*************************************************
			 	 CONFIGURATION
 *************************************************/

#define GPIO_ALTFN_AF05	5

#define SPI1_PORT			GPIOA
#define SPI1_PIN_NSS		GPIO_PIN_NO_4
#define SPI1_PIN_SCK 		GPIO_PIN_NO_5
#define SPI1_PIN_MISO		GPIO_PIN_NO_6
#define SPI1_PIN_MOSI		GPIO_PIN_NO_7

/* CS is connected to PE3 */
#define GYRO_CS_PORT		GPIOE
#define GYRO_PIN_CS			GPIO_PIN_NO_3

#define GYRO_PORT			SPI1_PORT
#define GYRO_SPI			SPI1

#define BUTTON_PORT			GPIOA
#define BUTTON_PIN			GPIO_PIN_NO_0

void Gyro_Control(uint8_t EnOrDis) {
	SPI_PeriControl(GYRO_SPI, EnOrDis);
	GPIO_WriteToOutputPin(GYRO_CS_PORT, GYRO_PIN_CS, !EnOrDis);
}

void System_Init (void) {
	/*
	 * SysTick
	 */
	SysTick_Init(ENABLE);

	/*
	 * SPI
	 */
	GPIO_Handle_t SPI_Pins;

	SPI_Pins.pGPIOx = SPI1_PORT;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF05;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI1_PIN_SCK;
	GPIO_Init(&SPI_Pins);

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI1_PIN_MISO;
	GPIO_Init(&SPI_Pins);

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI1_PIN_MOSI;
	GPIO_Init(&SPI_Pins);

//	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GYRO_PIN_CS;
//	SPI_Pins.pGPIOx = GYRO_CS_PORT;
//	GPIO_Init(&SPI_Pins);

	GPIO_Handle_t gyro_cs_pin;
//	memset(&gyro_cs_pin, 0, sizeof(gyro_cs_pin));
	gyro_cs_pin.pGPIOx = GYRO_CS_PORT;
	gyro_cs_pin.GPIO_PinConfig.GPIO_PinNumber = GYRO_PIN_CS;
	gyro_cs_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gyro_cs_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gyro_cs_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gyro_cs_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_Init(&gyro_cs_pin);

	SPI_Handle_t SPI1handle;
	SPI1handle.pSPIx = GYRO_SPI;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;	// data captured at the falling edge
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;	// data idle state is low
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN;		// hardware slave menadgment
	SPI1handle.SPIConfig.SPI_LSBFIRST = SPI_LSBFIRST_MSB;
	SPI_Init(&SPI1handle);

	//button configurations
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = BUTTON_PORT;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = BUTTON_PIN;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	//push pull
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioButton);

	/*
	 * LED pin
	 */

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	//push pull
//	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;	//open drain
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GpioLed);
}


void Gyro_Init () {
	if ((Gyro_ReadReg(GYRO_REG_WHOAMI) != 0xd3 )) {
		//ERROR
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
		simple_delay(MILLIS(500));

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
		simple_delay(MILLIS(500));

		return ;
	}
	/*
	 1. Write CTRL_REG2
	 2. Write CTRL_REG3
	 3. Write CTRL_REG4
	 4. Write CTRL_REG6
	 5. Write REFERENCE
	 6. Write INT1_THS
	 7. Write INT1_DUR
	 8. Write INT1_CFG
	 9. Write CTRL_REG5
	 10. Write CTRL_REG1
	 */
	Gyro_Config_t gyro;
	gyro.DataRate = 0x00;
	gyro.Bandwidth = 0x00;
	gyro.Zen = 1;
	gyro.Yen = 1;
	gyro.Xen = 1;

	uint8_t cmd = 0;
	// CTRL_REG1
	cmd |= (gyro.DataRate << 0);
	cmd |= (gyro.Bandwidth << 2);
	cmd |= (0x01 << 4); /* gyro.PD_Mode */
	cmd |= (gyro.Zen << 5);
	cmd |= (gyro.Yen << 6);
	cmd |= (gyro.Xen << 7);

	Gyro_SendData(GYRO_REG_CTRL1, &cmd, 1);

//	uint8_t Gyro_Mode_Normal = 0b00001111; //todo
	//Reverse_bits(&Gyro_Mode_Normal);
//	Gyro_SendData(GYRO_REG_CTRL1, &Gyro_Mode_Normal, sizeof(Gyro_Mode_Normal));
//	Gyro_SendByte (Gyro_Mode_Normal);

	/*  The device is provided with a STATUS_REG which should be polled to check when a new
		set of data is available. The reads should be performed as follows
	 1. Read STATUS_REG
	 2. If STATUS_REG(3) = 0, then go to 1
	 3. If STATUS_REG(7) = 1, then some data have been overwritten
	 4. Read OUT_X_L
	 5. Read OUT_X_H
	 6. Read OUT_Y_L
	 7. Read OUT_Y_H
	 8. Read OUT_Z_L
	 9. Read OUT_Z_H
	 10. Data processing
	 11. Go to 1

	 The check performed at step 3 allows understanding whether the reading rate is adequate
	compared to the data production rate. In case one or more angular rate samples have been
	overwritten by new data, because of an insufficient reading rate, the ZYXOR bit of
	STATUS_REG is set to 1.
	 The overrun bits are automatically cleared when all the data present inside the device have
	been read and new data have not been produced in the meantime
	 */
}

/*************************************************

 *************************************************/

/*
 * todo - change bit layout setting mechanism
 */
uint8_t Gyro_DataInfo (uint8_t ReadorWrite, uint8_t IncOrCont, uint8_t Addr) {
	uint8_t temp = 0;

    temp |= (Addr << (0));
    Reverse_bits(&temp);
    temp |= (ReadorWrite << (0));
    temp |= (IncOrCont << (1));
    Reverse_bits(&temp);

    return temp;
}

/*
 *
 */
uint8_t Gyro_SendByte (uint8_t data) {

	uint8_t read;

	SPI_SendData(GYRO_SPI, &data , 1);
	SPI_ReceiveData(GYRO_SPI, &read, 1);

	while (SPI_GetFlagStatus(GYRO_SPI, SPI_BUSY_FLAG));

	return read;
}

/*
 *
 */

void Gyro_SendData (uint8_t Addr , uint8_t* Data, uint16_t DataLen) {
	Gyro_Control(ENABLE);

	/* send info about the package */
	uint8_t package_info;
	if (DataLen > 1U) {
		package_info = Gyro_DataInfo(GYRO_WRITE, GYRO_ADDR_INC, Addr);
	} else {
		package_info = Gyro_DataInfo(GYRO_WRITE, GYRO_ADDR_CONST, Addr);
	}

	Gyro_SendByte(package_info);

	/* recieve the data*/
	while (DataLen > 0U) {
		Gyro_SendByte(*Data);
		DataLen--;
		Data++;
	}

	Gyro_Control(DISABLE);
}


void Gyro_ReadData (uint8_t Addr , uint8_t* pRxBuff, uint16_t DataLen) {
	Gyro_Control(ENABLE);

	/* send info about the package */
	uint8_t package_info;
	if (DataLen > 1U) {
		package_info = Gyro_DataInfo(GYRO_READ, GYRO_ADDR_INC, Addr);
	} else {
		package_info = Gyro_DataInfo(GYRO_READ, GYRO_ADDR_CONST, Addr);
	}

	Gyro_SendByte(package_info);

	/* recieve the data*/
	while (DataLen > 0U) {
		*pRxBuff = Gyro_SendByte(DUMMY_BYTE);
		DataLen--;
		pRxBuff++;
	}

	Gyro_Control(DISABLE);
}

/*
 * read value of the register
 */
uint8_t Gyro_ReadReg (uint8_t Addr) {
	uint8_t reg_value = 0;
	uint8_t dummy_byte = 0xff;
	uint8_t dummy_read;

	Gyro_Control(ENABLE);

	/* send info about the package */
	uint8_t package_info = Gyro_DataInfo(GYRO_READ, GYRO_ADDR_CONST, Addr);
	SPI_SendData(GYRO_SPI, &package_info , 1);
	SPI_ReceiveData(GYRO_SPI, &dummy_read, 1);

	while (SPI_GetFlagStatus(GYRO_SPI, SPI_BUSY_FLAG));
	/* Generate clock for the slave */
	SPI_SendData(GYRO_SPI, &dummy_byte, 1);
	SPI_ReceiveData(GYRO_SPI, &reg_value, 1);

	while (SPI_GetFlagStatus(GYRO_SPI, SPI_BUSY_FLAG));

	Gyro_Control(DISABLE);

	return reg_value;
}


// Read angular rate data
void Gyro_ReadAxis(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];

    // Read all 6 bytes of data (X, Y, Z each have LSB and MSB)
    Gyro_ReadData(GYRO_REG_OUT_X_L | 0x80, data, 6);

    // Combine the bytes into 16-bit values
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
}

/*
int main(void)
{
	System_Init();
	simple_delay(MILLIS(100));
	Gyro_Init();

	int16_t x,y,z;

	while (1) {
		Gyro_ReadAxis (&x, &y, &z);

        GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, (abs(x) > 15));
        GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, (abs(y) > 15));
        GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_14, (abs(z) > 15));

		simple_delay(MILLIS(20));
	}

	return 0;
}
*/
