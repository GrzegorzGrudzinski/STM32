/*
 * pwm_proba.c
 *
 *  Created on: Mar 2, 2025
 *      Author: grudz
 */

#include <string.h>
#include "stm32f411xx.h"

/*
 *
CH8 - PA7
CH7 - PA5
CH5 - PA6
CH4 - PE3
 */

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
//	memset(&SPI_Pins, 0, sizeof(SPI_Pins));
	SPI_Pins.pGPIOx = SPI1_PORT;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF05;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI1_PIN_SCK;
	GPIO_Init(&SPI_Pins);

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI1_PIN_MISO;
	GPIO_Init(&SPI_Pins);

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI1_PIN_MOSI;
	GPIO_Init(&SPI_Pins);

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
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;	// data captured at the falling edge
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;	// data idle state is low
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN;		// hardware slave menadgment
	SPI1handle.SPIConfig.SPI_LSBFIRST = SPI_LSBFIRST_MSB;
	SPI_Init(&SPI1handle);


	//button configurations
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;	//push pull
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioButton);
}


/*
  bit 0: RW
 bit. When 0, the data DI(7:0) is written to the device. When 1, the data DO(7:0)
from the device is read. In the latter case, the chip will drive SDO at the start of bit 8.
 bit 1: MS
 bit. When 0, the address remains unchanged in multiple read/write commands.
When 1, the address will be auto-incremented in multiple read/write commands.
 bit 2-7: address AD(5:0). This is the address field of the indexed register.
 bit 8-15: data DI(7:0) (write mode). This is the data that will be written to the device (MSb
first).
 bit 8-15: data DO(7:0) (read mode). This is the data that will be read from the device (MSb
first)
 */
#define GYRO_BIT_RW		0
#define GYRO_BIT_MS		1
#define GYRO_BIT_ADDR5	2
#define GYRO_BIT_ADDR4	3
#define GYRO_BIT_ADDR3	4
#define GYRO_BIT_ADDR2	5
#define GYRO_BIT_ADDR1	6
#define GYRO_BIT_ADDR0	7
#define GYRO_BIT_DATA7	8
#define GYRO_BIT_DATA6	9
#define GYRO_BIT_DATA5	10
#define GYRO_BIT_DATA4	11
#define GYRO_BIT_DATA3	12
#define GYRO_BIT_DATA2	13
#define GYRO_BIT_DATA1	14
#define GYRO_BIT_DATA0	15

#define GYRO_READ		1
#define GYRO_WRITE		0

#define GYRO_ADDR_INC		1
#define GYRO_ADDR_CONST 	0

#define GYRO_REG_WHOAMI		0x0F
#define GYRO_REG_CTRL1		0x20
#define GYRO_REG_CTRL2		0x21
#define GYRO_REG_CTRL3		0x22
#define GYRO_REG_CTRL4		0x23
#define GYRO_REG_CTRL5		0x24
#define GYRO_REG_REFERENCE	0x25
#define GYRO_REG_OUT_TEMP	0x26
#define GYRO_REG_STATUS		0x27
#define GYRO_REG_OUT_X_L	0x28
#define GYRO_REG_OUT_X_H	0x29
#define GYRO_REG_OUT_Y_L	0x2A
#define GYRO_REG_OUT_Y_H	0x2B
#define GYRO_REG_OUT_Z_L	0x2C
#define GYRO_REG_OUT_Z_H	0x2D
#define GYRO_REG_FIFIO_CTRL	0x2E
#define GYRO_REG_FIFIO_SRC	0x2F
#define GYRO_REG_INT1_CFG	0x30
#define GYRO_REG_INT1_SRC	0x31
#define GYRO_REG_INT1_THS_XH	0x32
#define GYRO_REG_INT1_THS_XL	0x33
#define GYRO_REG_INT1_THS_YH	0x34
#define GYRO_REG_INT1_THS_YL	0x35
#define GYRO_REG_INT1_THS_ZH	0x36
#define GYRO_REG_INT1_THS_ZL	0x37
#define GYRO_REG_INT1_DURATION	0x38

#define REVERSE_BIT_POS(X)	(7-X)

void Reverse_bits (uint8_t* value) {
	uint8_t rev_temp = 0;
	for (int i=sizeof(*value)*7; i>=0; i--) {
		rev_temp |= ((*value >> i) & 0x1) << (7-i);
	}

	*value = rev_temp;
}

uint8_t Gyro_DataInfo (char ReadorWrite, char AddrIncOrNot, uint8_t Addr) {
	uint8_t temp = 0;

	temp |= (ReadorWrite << (GYRO_BIT_RW) );
	temp |= (AddrIncOrNot << (GYRO_BIT_MS) );
	temp |= (Addr << (GYRO_BIT_ADDR5) );

//	Reverse_bits(&temp);

	return temp;
}

#define BYTE_SIZE		8

/*
 *
 */
void Gyro_SendData (uint8_t Addr ,uint8_t* pTxBuff, uint32_t data_len) {
	/* send info about the package */
	char big_data = 0;
	if (data_len > 1) {
		big_data = GYRO_ADDR_INC;
	} else {
		big_data = GYRO_ADDR_CONST;
	}
	uint8_t package_info = Gyro_DataInfo(GYRO_WRITE, big_data, Addr);
	uint8_t dummy_read;


	Gyro_Control(ENABLE);

	SPI_SendData(GYRO_SPI, &package_info , 1);
	SPI_ReceiveData(GYRO_SPI, &dummy_read, 1);

	while (SPI_GetFlagStatus(GYRO_SPI, SPI_BUSY_FLAG));

	/* Send data */
	SPI_SendData(GYRO_SPI, pTxBuff, data_len);
	SPI_ReceiveData(GYRO_SPI, &dummy_read, 1);

	Gyro_Control(DISABLE);
}

/*
 *
 */
void Gyro_ReadData (uint8_t Addr ,uint8_t* pRxBuff, uint8_t INCorCONST ) {
	Gyro_Control(ENABLE);

	/* send info about the package */
	uint8_t package_info = Gyro_DataInfo(GYRO_READ, INCorCONST, Addr);
	uint8_t dummy_read;
	SPI_SendData(GYRO_SPI, &package_info , 1);
	SPI_ReceiveData(GYRO_SPI, &dummy_read, 1);

	/* Recieve data */
//	Gyro_Control(ENABLE);
	uint8_t dummy_byte = 0xff;
	SPI_SendData(GYRO_SPI, &dummy_byte, 1);
	SPI_ReceiveData(GYRO_SPI, pRxBuff, 1);

	Gyro_Control(DISABLE);
}

uint8_t Gyro_ReadReg (uint8_t Addr) {
	uint8_t reg_value = 0;
	uint8_t dummy_byte = 0xff;
	uint8_t dummy_read;

	Gyro_Control(ENABLE);

	/* send info about the package */
	uint8_t package_info = Gyro_DataInfo(GYRO_READ, GYRO_ADDR_CONST, Addr);
	SPI_SendData(GYRO_SPI, &package_info , 1);
	SPI_ReceiveData(GYRO_SPI, &dummy_read, 1);

	/* Recieve data */
	SPI_SendData(GYRO_SPI, &dummy_byte, 1);
	SPI_ReceiveData(GYRO_SPI, &reg_value, 1);

	Gyro_Control(DISABLE);

	return reg_value;
}
//uint8_t Gyro_ReadReg (uint8_t Addr) {
//	uint8_t reg_value = 0;
//	uint8_t dummy = 0xff;
//	uint8_t dummy_read;
//
//	Gyro_Control(ENABLE);
//
//	/* Send info about the package */
//	uint8_t package_info = Gyro_DataInfo(GYRO_READ, GYRO_ADDR_CONST, Addr);
//
//	// 1. Wyślij package_info
//	SPI_SendData(GYRO_SPI, &package_info, 1);
//
//	// 2. WYMUSZ dummy read, bo SPI jest pełne
////	while (! (SPI_GetFlagStatus(GYRO_SPI, SPI_RXNE_FLAG) == FLAG_RESET));
//	dummy_read = GYRO_SPI->DR.reg; // czyścimy śmiecia
//
//	// 3. Teraz wyślij dummy_byte
//	SPI_SendData(GYRO_SPI, &dummy, 1);
//
//	// 4. Poczekaj na dane
////	while (!SPI_GetFlagStatus(GYRO_SPI, SPI_RXNE_FLAG));
//	reg_value = GYRO_SPI->DR.reg; // to już prawdziwe dane
//
//	while (SPI_GetFlagStatus(GYRO_SPI, SPI_BUSY_FLAG));
//	Gyro_Control(DISABLE);
//
//	return reg_value;
//}
void Gyro_Init () {
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

uint8_t Gyro_Mode_Normal = 0b00001111;
//Reverse_bits(&Gyro_Mode_Normal);
Gyro_SendData(GYRO_REG_CTRL1, &Gyro_Mode_Normal, sizeof(Gyro_Mode_Normal));

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

int main(void)
{
//	uint8_t Data;
//	System_Init();
//	simple_delay(MILLIS(100));
//	Gyro_ReadData(0x0F, Data, GYRO_ADDR_CONST);
//

	System_Init();
//	simple_delay(MILLIS(100));

//	uint8_t test = Gyro_ReadReg(GYRO_REG_WHOAMI);
//	Gyro_Init();

	char hello[] = "Hello World!";
//	uint8_t dane[2];

//	Gyro_ReadData(GYRO_REG_WHOAMI, dane, GYRO_ADDR_CONST);

//	while (1) {
//		simple_delay(SECONDS(1));
//
//		Gyro_ReadData(GYRO_REG_OUT_X_L, dane, GYRO_ADDR_INC);
//
//		int16_t x = (int16_t)(dane[1] << 8 | dane[0]);
//		Gyro_Control(ENABLE);

//		SPI_SendData( GYRO_SPI,(uint8_t*) hello, strlen(hello));

//		while (SPI_GetFlagStatus(GYRO_SPI, SPI_BUSY_FLAG));
//
//		Gyro_Control(DISABLE);
//	}

//		/* main loop */
//
//		Gyro_ReadData
//		while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == HIGH);
		SPI_PeriControl(GYRO_SPI, ENABLE);
		SPI_SendData( GYRO_SPI,(uint8_t*) hello, strlen(hello));
//		SPI_PeriControl(GYRO_SPI, DISABLE);
		while (1) ;//{
//
//
//	}

	return 0;
}

