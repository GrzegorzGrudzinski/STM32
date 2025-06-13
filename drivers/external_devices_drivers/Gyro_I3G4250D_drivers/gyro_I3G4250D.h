/*
 * gyro_L3GD20.h
 *
 *  Created on: May 30, 2025
 *      Author: grudz
 */

#ifndef GYRO_I3G4250D_DRIVERS_GYRO_I3G4250D_H_
#define GYRO_I3G4250D_DRIVERS_GYRO_I3G4250D_H_

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

#define BYTE_SIZE		8


typedef struct
{
	uint8_t FIFO_Mode;
	uint8_t DataRate;
	uint8_t Bandwidth;
	uint8_t Power_Mode;
	uint8_t Zen;
	uint8_t Yen;
	uint8_t Xen;
//	uint8_t Filter_HighPass;
//	uint8_t Filter_LowPass;


} Gyro_Config_t;


void Gyro_Control(uint8_t EnOrDis);
void System_Init (void);
void Gyro_Init ();
uint8_t Gyro_DataInfo (uint8_t ReadorWrite, uint8_t IncOrCont, uint8_t Addr);
uint8_t Gyro_SendByte (uint8_t data);
void Gyro_SendData (uint8_t Addr , uint8_t* Data, uint16_t DataLen);
void Gyro_ReadData (uint8_t Addr ,uint8_t* pRxBuff, uint16_t DataLen);
uint8_t Gyro_ReadReg (uint8_t Addr);


#endif /* GYRO_I3G4250D_DRIVERS_GYRO_I3G4250D_H_ */
