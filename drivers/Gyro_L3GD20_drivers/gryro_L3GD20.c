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
 */

#define GPIO_ALTFN_AF05	5

#define SPI1_PORT			GPIOA
#define SPI1_PIN_NSS		GPIO_PIN_NO_4
#define SPI1_PIN_SCK 		GPIO_PIN_NO_5
#define SPI1_PIN_MISO		GPIO_PIN_NO_6
#define SPI1_PIN_MOSI		GPIO_PIN_NO_7

#define GYRO_SPI			SPI1


/*
 *
 */

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
		SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI2_PIN_NSS;
		GPIO_Init(&SPI_Pins);

		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI2_PIN_SCK;
		GPIO_Init(&SPI_Pins);

		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI2_PIN_MISO;
		GPIO_Init(&SPI_Pins);

		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = SPI2_PIN_MOSI;
		GPIO_Init(&SPI_Pins);

		SPI_Handle_t SPI1handle;
		SPI1handle.pSPIx = GYRO_SPI;
		SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
		SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
		SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
		SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
		SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;	// data captured at the falling edge
		SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;	// data idle state is low
		SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI;		// hardware slave menadgment
		SPI1handle.SPIConfig.SPI_LSBFIRST = SPI_LSBFIRST_LSB;

		SPI_Init(&SPI1handle);
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

uint8_t Gyro_DataInfo (char ReadorWrite, char AddrIncOrNot, uint8_t Addr) {
	uint8_t temp = 0;

	temp |= (ReadorWrite << GYRO_BIT_RW );
	temp |= (AddrIncOrNot << GYRO_BIT_MS);

	temp |= (Addr << GYRO_BIT_ADDR5 );

	return temp;
}

#define BYTE_SIZE		8
void Gyro_SendData (uint8_t ReadOrWrite,uint8_t Addr ,uint8_t* pTxBuff, uint32_t data_len) {
	/* send info about the package */
	char big_data = 0;
	if (data_len > 8) {
		big_data = GYRO_ADDR_INC;
	} else {
		big_data = GYRO_ADDR_CONST;
	}

	uint8_t package_info = Gyro_DataInfo(ReadOrWrite, big_data, Addr);

	SPI_SendData(GYRO_SPI, package_info , BYTE_SIZE);

	/* Send data */
	SPI_SendData(GYRO_SPI, pTxBuff, data_len);
}


int main(void)
{
	System_Init();

	char hello[] = "Hello World!";

	while (1) {
		/* main loop */
		while (! GPIO_ReadFromInputPin(BUTTON_PORT, BUTTON_PIN));
		simple_delay(MILLIS(100));

		SPI_PeriControl(SPI2, ENABLE);

		SPI_SendData( SPI2, hello, strlen(hello));

		while (! SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeriControl(SPI2, DISABLE);
	}

	return 0;
}

