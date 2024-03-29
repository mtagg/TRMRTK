/*
 * MC3479.h
 *
 *  Created on: Oct 9, 2022
 *      Author: mmmta
 *
 *      Header file for the MC3479 (Memsic Inc.) 3-axis Accelerometer Driver
 *
 *      To avoid large sections of comments, functions/methods will use descriptive names.
 *      Any code that references the data-sheet will have the page number of the datasheet provided:
 *      https://www.memsic.com/Public/Uploads/uploadfile/files/20220522/MC3479Datasheet(APS-048-0072v1.2).pdf
 */

#ifndef SRC_MC3479_H_
#define SRC_MC3479_H_

//Include HAL to allow passed-by-reference HAL objects
#include "stm32f1xx_hal.h"
//#include "main.h"


/*
 * Register Name/Address macros (p41):
 */
//OX00-0X04
#define MC3479_DEV_STAT			0x05
#define MC3479_INTR_CTRL		0x06
#define MC3479_MODE				0x07
#define MC3479_SR				0x08
#define MC3479_MOTION_CTRL		0x09
#define MC3479_FIFO_STAT		0x0A
#define MC3479_FIFO_RD_P		0x0B
#define MC3479_FIFO_WR_P		0x0C
#define MC3479_XOUT_EX_L		0x0D
#define MC3479_XOUT_EX_H		0x0E
#define MC3479_YOUT_EX_L		0x0F
#define MC3479_YOUT_EX_H		0x10 //Note that the data-sheet has a typo, this is the register for MSB***
#define MC3479_ZOUT_EX_L		0x11
#define MC3479_ZOUT_EX_H		0x12
#define MC3479_STATUS			0x13
#define MC3479_INTR_STAT		0x14
//0X15-0X1F - Reserved** (p41)
#define MC3479_CHIP_ID			0x18
#define MC3479_RANGE			0x20
#define MC3479_XOFFL			0x21
#define MC3479_XOFFH			0x22
#define MC3479_YOFFL			0x23
#define MC3479_YOFFH			0x24
#define MC3479_ZOFFL			0x25
#define MC3479_ZOFFH			0x26
#define MC3479_XGAIN			0x27
#define MC3479_YGAIN			0x28
#define MC3479_ZGAIN			0x29
//0X2A-0X2C - Reserved** (p42)
#define MC3479_FIFO_CTRL		0x2D
#define MC3479_FIFO_TH			0x2E
#define MC3479_FIFO_INTR		0x2F
#define MC3479_FIFO_CTRL2_SR2	0x30
#define MC3479_COMM_CTRL		0x31
//0X32 - Reserved** (p42)
#define MC3479_GPIO_CTRL		0x33
//0X34-3F - Reserved** (p42)
#define MC3479_TF_THRESH_LSB			0x40
#define MC3479_TF_THRESH_MSB			0x41
#define MC3479_TF_DB					0x42
#define MC3479_AM_THRESH_LSB			0x43
#define MC3479_AM_THRESH_MSB			0x44
#define MC3479_AM_DB					0x45
#define MC3479_SHK_THRESH_LSB			0x46
#define MC3479_SHK_THRESH_MSB			0x47
#define MC3479_PK_P2P_DUR_THRESH_LSB	0x48
#define MC3479_PK_P2P_DUR_THRESH_MSB	0x49
#define MC3479_TIMER_CTRL				0x4A
#define MC3479_RD_CNT					0x4B
//0X4C-0X50 - RESERVED** (p43)



/*
 * INTERRUPT ENABLE REGISTER (0x06) MACROS (p45)
 */
#define TILT_INT_EN		0x01
#define FLIP_INT_EN		0x02
#define ANYM_INT_EN		0x04
#define SHAKE_INT_EN	0x08
#define TILT_35_INT_EN	0x10
#define AUTO_CLR_EN		0x40
#define ACQ_INT_EN 		0x80

/*
 * MODE REGISTER (0x07) MACROS (p46)
 */
#define SLEEP			0x00
#define WAKE			0x01
#define STANDBY			0x03
#define I2C_WDT_NEG_EN	0x10
#define I2C_WDT_POS_EN 	0x20



/*
 * SAMPLE RATE REGISTER (0X08) MACROS  (p47)
 */
#define RATE0_50Hz   0x08
#define RATE1_100Hz  0x09
#define RATE2_125Hz  0x0A
#define RATE3_200Hz  0x0B
#define RATE4_250Hz  0x0C
#define RATE5_500Hz  0x0D
#define RATE6_1000Hz 0x0E
#define RATE7_2000Hz 0x0F

/*
 * MOTION CONTROL REGISTER (0X09) MACROS (P48)
 * *****************************************************
 * TODO: FILL OUT FROM DATASHEET
 */



/*
 * FIFO STATUS REGISTER (0X0A) MACROS (P49)
 */
#define FIFO_EMPTY	0x01
#define FIFO_FULL	0x02
#define FIFO_THRESH	0x04

/*
 * FIFO READ POINTER REGISTER (0X0B) MACROS (P50)
 * *****************************************************
 * TODO: DO WE EVEN NEED THIS?
 */


/*
 * FIFO WRITE POINTER REGISTER (0X0B) MACROS (P50)
 * *****************************************************
 * TODO: DO WE EVEN NEED THIS?
 */

/*
 * ACCELEROMETER OUTPUT DATA REGISTERS (0X0D-0X12) MACROS (P52)
 * *****************************************************
 * Not applicable, data is received from these registers as 16-bit 2's compliment
 */

/*
 * STATUS REGISTER (0X13) MACROS (P53)
 * RECOMMENDED TO USE REGISTER 0X14 FOR READING INTERRUPT STATUS WHEN POLLING IS USED
 * THE FLAG BITS IN REGISTER 0X13 CAN FLIP QUICKLY
 */
#define TILT_FLAG		0x01
#define FLIP_FLAG		0x02
#define ANYM_FLAG		0x04
#define SHAKE_FLAG		0x08
#define TILT_35_FLAG	0x10
#define FIFO_FLAG		0x20
#define NEW_DATA		0x80

/*
 * INTERRUPT STATUS REGISTER (0X14) MACROS (P54)
 * ALL INTERRUPTS ARE CLEARED AFTER BEING READ
 */
#define TILT_INT		0x01
#define FLIP_INT		0x02
#define ANYM_INT		0x04
#define SHAKE_INT		0x08
#define TILT_35_INT		0x10
#define FIFO_INT		0x20
#define ACQ_INT			0x80


/*
 * RANGE AND SCALE CONTROL REGISTER (0X20) MACROS (P55)
 */

#define ADD_2G_RESOLUTION	0x00
#define ADD_4G_RESOLUTION	0x01
#define ADD_8G_RESOLUTION	0x02
#define ADD_16G_RESOLUTION	0x03
#define ADD_12G_RESOLUTION	0x04
#define LPF_ENABLE 			0x08
#define LPF_BW1				0x10
#define LPF_BW2				0x20
#define LPF_BW3				0x30
#define LPF_BW5				0x50


/*
 * X-AXIS DIGITAL OFFSET REGISTERS (0X21-0X22) MACROS (P56)
 * PERFORM WRITES AS: READ-MODIFY-WRITE COMMANDS TO MAINTAIN UNCHANGED SETTINGS
 * *****************************************************
 * TODO: DO WE NEED TO MODIFY OFFSETS INTO THE DATA OUTPUT????
 */

/*
 * Y-AXIS DIGITAL OFFSET REGISTERS (0X23-0X24) MACROS (P57)
 * PERFORM WRITES AS: READ-MODIFY-WRITE COMMANDS TO MAINTAIN UNCHANGED SETTINGS
 * *****************************************************
 * TODO: DO WE NEED TO MODIFY OFFSETS INTO THE DATA OUTPUT????
 */

/*
 * Z-AXIS DIGITAL OFFSET REGISTERS (0X25-0X26) MACROS (P58)
 * PERFORM WRITES AS: READ-MODIFY-WRITE COMMANDS TO MAINTAIN UNCHANGED SETTINGS
 * *****************************************************
 * TODO: DO WE NEED TO MODIFY OFFSETS INTO THE DATA OUTPUT????
 */


/*
 * X-AXIS DIGITAL GAIN REGISTERS (0X22 & 0X27) MACROS (P59)
 * PERFORM WRITES AS: READ-MODIFY-WRITE COMMANDS TO MAINTAIN UNCHANGED SETTINGS
 * NOTE THAT REGISTER 0X22 BIT-7 IS THE 8TH BIT OF GAIN
 * *****************************************************
 * TODO: DO WE NEED TO MODIFY GAIN??
 */

/*
 * Y-AXIS DIGITAL GAIN REGISTERS (0X24 & 0X28) MACROS (P60)
 * PERFORM WRITES AS: READ-MODIFY-WRITE COMMANDS TO MAINTAIN UNCHANGED SETTINGS
 * NOTE THAT REGISTER 0X24 BIT-7 IS THE 8TH BIT OF GAIN
 * *****************************************************
 * TODO: DO WE NEED TO MODIFY GAIN??
 */

/*
 * Z-AXIS DIGITAL GAIN REGISTERS (0X26 & 0X29) MACROS (P61)
 * PERFORM WRITES AS: READ-MODIFY-WRITE COMMANDS TO MAINTAIN UNCHANGED SETTINGS
 * NOTE THAT REGISTER 0X26 BIT-7 IS THE 8TH BIT OF GAIN
 * *****************************************************
 * TODO: DO WE NEED TO MODIFY GAIN??
 */

/*
 * FIFO CONTROL REGISTER (0X2D) MACROS (P62-p63)
 */
#define FIFO_EMPTY_INT_EN	0x01
#define FIFO_FULL_INT_EN	0x02
#define FIFO_TH_INT_EN		0x04
#define COMB_INT_EN			0x08
#define FIFO_RESET			0x10
#define FIFO_EN				0x20
#define FIFO_MODE			0x40


/*
 * FIFO THRESHOLD REGISTER (0X2E) MACROS (P64)
 */
#define FIFO_TH_MIN			0x01
#define FIFO_TH_25PERCENT	0x08
#define FIFO_TH_50PERCENT	0x10
#define FIFO_TH_DEFAULT		0x10
#define FIFO_TH_75PERCENT	0x18
#define FIFO_TH_MAX			0x1F

/*
 * FIFO INTERRUPT STATUS REGISTER (0X2F) MACROS (P65)
 * THIS REGISTER REQUIRES BITS FROM REGISTER 0X2D TO BE SET
 * ****nOTE THAT THERE IS A TYPO ON THE DATASHEET, IT SHOULD BE REGISTER 0X2D, NOT 2B
 */
#define FIFO_EMPTY_INT_FLAG		0x01
#define FIFO_FULL_INT_FLAG		0x02
#define FIFO_THRESH_INT_FLAG	0x04

/*
 * FIFO CONTROL REGISTER 2, SAMPLE RATE REGISTER 2 (0X30) MACROS (P66-P67)
 * CONTROLS THE BEHAVIOR WHEN USING FIFO BURST MODE
 */

#define ENABLE_WRAP_N		0x10
#define SELECT_WRAP_ADDR 	0x20 //includes accel flags/interrupts
#define FIFO_BURST			0x80 //Include to allow multiple samples per burst

/*
 * COMMUNICATION CONTROL REGISTER (0X31) MACROS (P68)
 * CONFIGURATION FOR SERIAL COMS - defaults are all set to disabled (0x00)
 * *****************************************************
 * TODO: DO WE NEED THIS REGISTER TO BE CONFIGURED AT ALL?
 */
#define INT1_INT2_REQ_SWAP	0x10
#define SPI_EWIRE_EN		0x20
#define INDIV_INTR_CLR		0x40

/*
 * GPIO CONTROL REGISTER (0X33) MACROS (P69 - NICE)
 * *****************************************************
 * TODO: DETERMINE THE CONFIGURATION WE WANT TO USE FOR EACH ACCELEROMETER
 */
#define GPIO1_INTN1_IAH 	0x04
#define GPIO1_INTN1_IPP		0x08
#define GPIO2_INTN2_IAH		0x40
#define GPIO2_INTN2_IPP		0x80

/*
 * TILT/FLIP THRESHOLD REGISTER (0X40-0X41) (P70)
 * CONFIGURES A 15-BIT THRESHOLD (8 BITS OF 0X40 AND 7 BITS OF 0X41)
 * VALUE FOR TILT/FLIP WHERE AN INTERRUPT WILL TRIGGER IN
 * REGISTER 0X09 BIT-5 (FLAT/FLIP) OR 0X14 BIT-4 (TILT-35)
 */


/* TILT/FLIP DEBOUNCE REGISTER (0X42) (P71)
 * 8 BIT PROGRAMMED DURATION OF A TILT/FLIP
 * IF DURATION IS GREATER THAN VALUE, INTERRUPT SET IN REGISTER 0X14 BIT 0 AND 1
 */

/*
 * ANYMOTION THRESHOLD REGISTERS (0X43-0X44) (P72)
 */

/*
 *  ANYMOTION DEBOUNCE REGISTER (0X45) (P73)
 */

/*
 * SHAKE THRESHOLD REGISTERS (0X46-0X47) (P74)
 */

/*
 *  SHAKE DURATION, P2P REGISTERS (0X48-0X49) (P75)
 */

/*
 * TIMER CONTROL REGISTER (0X4A) (P76)
 * ******************************************************
 * TODO: DO WE NEED TO UYSE TILT-35 FOR OUR USE-CASE?
 */

/*
 * READ COUNT REGISTER (0X4B) (P77)
 * USED WHEN REGISTER 0X30 BIT 7 IS SET (FOR FIFO BURST)
 * ******************************************************
 * TODO: ARE WE USING FIFO BURST READS?
 */


/*
 * MC3479 I2C Communication Macros (p35)
 * I2C may run up to 1MHz
 * To communicate via SPI:
 * 1. 	The host generates START conditions
 * 2.	The device id << (SLL) is sent and acklowledged
 * 3. 	The Reg address is sent and acked
 * 4a.	If writing to reg, then 8bit data is sent/acked
 * 4b.	If reading the reg, a RESTART condition is set, followed by a device (ID<<7)+1 for a read, and the data is sent back to host.
 * 5. 	STOP condition is set by host
 *
 * NOTE: pin3 for DOUT_A6 can be pulled to GND or VCC to determine Device ID
 * TODO GET ALL MACROS FROM DATASHEET
 */

//DEV ID 1 when DOUT_A6(pin3) is pulled down to GND (p9-10,35)
#define I2C_DEV_ID_1 		0x4C
#define I2C_DEV_ID_WRITE_1	0x98
#define I2C_DEV_ID_READ_1	0x99

//DEV ID 2 when DOUT_A6(pin3) is pulled down to VCC (p9-10,35)
#define I2C_DEVICE_ID_2 	0x6C
#define I2C_DEV_ID_WRITE_2	0xD8
#define I2C_DEV_ID_READ_2	0xD9

/*
 * MC3479 SPI Communication Macros
 * SPI may run up to 10MHz.
 * To communicate via SPI:
 * 1.	CSN is asserted LOW
 * 2.	first Byte (MSB) is 0b0xxx_xxxx (read) or 0x1xxx_xxxx(write)
 * 			the x's are the reg addr  (MSB first)
 * 3a.	second byte is the data in(to reg (MSB first) for a reg write
 * 3b 	second byte is 0x00 for read, third byte is read back from register
 * 4. Data is shifted on CLK rising edge
 * TODO do we want to configure burst writes (p38)
 * TODO GET ALL MACROS FROM DATASHEET
 */

#define _SPI_COM_ENABLED
#define SPIwrite_REG		0x00 //bit-wise OR this with the reg number for register write
#define SPIread_REG			0x80 //bit-wise OR this with the reg number for register read
#define SPIread_BYTE2		0x00
#define REG_BYTES_LEN		1

class MC3479Class{
public:
/*
 * Class Variables
 */
	SPI_HandleTypeDef  * _SPI1;
	I2C_TypeDef		   * _I2C1;
	GPIO_TypeDef 	   * _CSN_GPIO;
	uint16_t _CSN_PIN;
	uint8_t I2C_DEVICE_ID;
	uint8_t I2C_writeId;
	uint8_t I2C_readId;


/*
 * MC3479Class Function-Headers
 */

#ifndef _SPI_COM_ENABLED
	// Set the MC3479's I2C object and initialize the device I
	bool setSerialI2C(I2C_TypeDef * i2c, uint8_t devId);
	// Read from a register using I2C
	uint8_t I2C_readRegister(uint8_t reg, uint8_t* data);
	// Write to a register using I2C
	uint8_t I2C_writeRegister(uint8_t reg, uint8_t data);
#endif

	// Set the MC3479's SPI object
	bool setSerialSPI(SPI_HandleTypeDef * spi,GPIO_TypeDef * csn_GPIO, uint16_t csn_PIN );
	// Read from a register using SPI
	bool SPI_readRegister(uint8_t reg, uint8_t* data);
	// Write to a register using SPI
	uint8_t SPI_writeRegister(uint8_t reg, uint8_t data);
	// Read from a register using SPI
	bool burstSPI_readRegister(uint8_t reg, uint8_t* data, uint8_t reg_count);
	// Write to a register using SPI
	bool burstSPI_writeRegister(uint8_t reg, uint8_t data, uint8_t reg_count);

	// Perform the initial MC3479 hard-coded configuration
	void configAccelerometer();
	// Set the accelerometer's sample rate from 50-2000Hz
	uint8_t setSampleRate(uint8_t rate);
	bool getXYZ(uint8_t* xData, uint8_t* yData, uint8_t* zData);
	uint8_t getMotionFlagStatus();
	uint8_t getMotionIrqStatus();
	bool clearMotionIrqStatus();
	uint8_t getFifoIrqStatus();



	/*
	 * TODO: Additional headers
	 * get FIFO pointer??
	 * get FIFO status??
	 * Sleep Mode enable???
	 * Idle Mode Enable
	 * WakeUp
	 */

};





//Initialize a class to be used whenever this library is used (allows for easier referencing, similar to Arduino libraries)
extern MC3479Class MC3479;






#endif /* SRC_MC3479_H_ */
