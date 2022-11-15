/*
 * MC3479.cpp
 *
 *  Created on: Oct 9, 2022
 *      Author: mmmta
 */
//TODO: Add try/except or assertion clauses to all methods
#include "MC3479.h"


MC3479Class MC3479;



// Set the MC3479's SPI object
bool MC3479Class::setSerialSPI(SPI_HandleTypeDef * spi,GPIO_TypeDef * csn_GPIO, uint16_t csn_PIN )
{
	this->_SPI1 = spi;
	this->_CSN_GPIO = csn_GPIO;
	this->_CSN_PIN = csn_PIN;
	return 1; // Return Success
}

// Read from a register using SPI
bool MC3479Class::SPI_readRegister(uint8_t reg,  uint8_t* data)
{

	uint8_t spiBytes[2];
	spiBytes[0] = SPIread_REG && reg;
	spiBytes[1] = SPIread_BYTE2;
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->_SPI1, spiBytes, sizeof(spiBytes), 10);
	HAL_SPI_Receive(this->_SPI1, data, REG_BYTES_LEN, 10);
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
	return 1;
}

// Write to a register using SPI
uint8_t MC3479Class::SPI_writeRegister(uint8_t reg, uint8_t data)
{
	// Write data to reg:
	uint8_t spiBytes[2];
	spiBytes[0] = SPIwrite_REG && reg;
	spiBytes[1] = data;
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->_SPI1, spiBytes, sizeof(spiBytes), 10);
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);

	// Read back the register and return the bytes:
	uint8_t regReadBack = 0;
	MC3479Class::SPI_readRegister(reg, &regReadBack);
	return regReadBack;
}

// Read from a register using SPI
bool MC3479Class::burstSPI_readRegister(uint8_t reg, uint8_t* data, uint8_t reg_count)
{
	uint8_t spiBytes[2];
	spiBytes[0] = SPIread_REG && reg;
	spiBytes[1] = SPIread_BYTE2;
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->_SPI1, spiBytes, sizeof(spiBytes), 10);

	for (uint8_t i=0; i<reg_count; i++)
	{
		//TODO: Validate that data[0] will be Xdata LSB
		HAL_SPI_Receive(this->_SPI1, &data[i], REG_BYTES_LEN, 10);
	}
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
	return 1;

}

// Burst-Write to a register using SPI
bool MC3479Class::burstSPI_writeRegister(uint8_t reg, uint8_t data, uint8_t reg_count)
{
	// TODO: Decide if this is useful.
	// Can only see us needing this function in the case where we need to set multiple registers on the fly.
	// TODO: Possibly could use this for configuration if we run out of flash???
	return 1;
}
#ifndef _SPI_COM_ENABLED
// Set the MC3479's I2C object and initialize the device I
bool MC3479Class::setSerialI2C(I2C_TypeDef * i2c, uint8_t devId)
{
	this->_I2C1 = i2c;
	this->I2C_DEVICE_ID = devId;
	this->I2C_writeId = (I2C_DEVICE_ID << 1) | 0x00;
	this->I2C_readId = (I2C_DEVICE_ID << 1) | 0x01;
	return 1; // Return Success
}
// Read from a register using I2C
uint8_t MC3479Class::I2C_readRegister(uint8_t reg, uint8_t* data)
{
	//HAL_I2C_Master_Transmit(_I2C1, I2C_DEVICE_ID, data, REG_BYTES_LEN, 10)
	return 1;
}
// Write to a register using I2C
uint8_t MC3479Class::I2C_writeRegister(uint8_t reg, uint8_t data)
{
	return 1;
}
#endif

// Perform the initial MC3479 hard-coded configuration
void MC3479Class::configAccelerometer(){
	uint8_t data;


#ifdef _SPI_COM_ENABLED
		//Configuration using SPI:

		// Register 0x06 (interrupt enable)
		data = 0xFF & ACQ_INT_EN; //only activates interrupts after each sample
		MC3479Class::SPI_writeRegister(MC3479_INTR_CTRL, data);

		// Register 0x07 (MODE)
		data = 0xFF & WAKE; // clocks running, X,Y,Z axis sampled @ data rate
		MC3479Class::SPI_writeRegister(MC3479_MODE, data);

		// Register 0x08 (Sample  Rate)
		data = 0xFF & RATE1_100Hz; // sample x,y,z @ 100Hz
		MC3479Class::SPI_writeRegister(MC3479_SR, data);

		// Register 0x09 (Motion Control)
		data = 0xFF & 0x00; //No motion detection enabled Z-axis positive through top of package
		MC3479Class::SPI_writeRegister(MC3479_MOTION_CTRL, data);

		// Register 0x20 (Range Select Control)
		data = 0xFF & 0x00; // No resolution range change, no LPF
		MC3479Class::SPI_writeRegister(MC3479_RANGE, data);

		// Register 0x21 (X-offset lSB)
		MC3479Class::SPI_readRegister(MC3479_XOFFL, &data);
		data = data & 0xFF; //no offset
		MC3479Class::SPI_writeRegister(MC3479_XOFFL, data);

		// Register 0x22 (X-offset MSB)
		MC3479Class::SPI_readRegister(MC3479_XOFFH, &data);
		data = data & 0xFF; //no offset
		MC3479Class::SPI_writeRegister(MC3479_XOFFH, data);

		// Register 0x23 (Y-offset LSB)
		MC3479Class::SPI_readRegister(MC3479_YOFFL, &data);
		data = data & 0xFF; //no offset
		MC3479Class::SPI_writeRegister(MC3479_YOFFL, data);

		// Register 0x24 (Y-offset MSB)
		MC3479Class::SPI_readRegister(MC3479_YOFFH, &data);
		data = data & 0xFF; //no offset
		MC3479Class::SPI_writeRegister(MC3479_YOFFH, data);

		// Register 0x25 (Z-offset LSB)
		MC3479Class::SPI_readRegister(MC3479_ZOFFL, &data);
		data = data & 0xFF; //no offset
		MC3479Class::SPI_writeRegister(MC3479_ZOFFL, data);

		// Register 0x26 (Z-offset MSB)
		MC3479Class::SPI_readRegister(MC3479_ZOFFH, &data);
		data = data & 0xFF; //no offset
		MC3479Class::SPI_writeRegister(MC3479_ZOFFH, data);

		// Register 0x27 (X Gain)
		MC3479Class::SPI_readRegister(MC3479_XGAIN, &data);
		data = data & 0xFF; //no GAIN
		MC3479Class::SPI_writeRegister(MC3479_XGAIN, data);

		// Register 0x28 (Y Gain)
		MC3479Class::SPI_readRegister(MC3479_YGAIN, &data);
		data = data & 0xFF; //no GAIN
		MC3479Class::SPI_writeRegister(MC3479_YGAIN, data);

		// Register 0x29 (Z Gain)
		MC3479Class::SPI_readRegister(MC3479_ZGAIN, &data);
		data = data & 0xFF; //no GAIN
		MC3479Class::SPI_writeRegister(MC3479_ZGAIN, data);

		// RegisteO 0x2D (FIFO Control)
		data = 0XFF & FIFO_TH_INT_EN & FIFO_FULL_INT_EN; // FIFO TH/Full IRQ set on INTN2 pin.
		MC3479Class::SPI_writeRegister(MC3479_FIFO_CTRL, data);

		// Register 0x2E (FIFO Threshold)
		data = 0xFF & 0x10; //FIFO IRQ threshold set to 50%
		MC3479Class::SPI_writeRegister(MC3479_FIFO_TH, data);

		// Register 0x30 (FIFO Control 2, Sample Rate 2)
		//Burst-read cycle that includes XOUT[15:0], YOUT[15:0],
		//ZOUT[15:0], annd NOTTTT: STATUS[7:0], and INTR_STATUS[7:0]:
		data = 0xFF & 0x00;
		MC3479Class::SPI_writeRegister(MC3479_FIFO_CTRL2_SR2, data);

		// Register 0x31 (Communication Control)
		data = 0xFF & 0x00; //0x14 interrupts are cleared simultaneously, 4bit SPI, default Interrupt pins
		MC3479Class::SPI_writeRegister(MC3479_COMM_CTRL, data);

		// Register 0x33 (GPIO Control)
		data = 0xFF & GPIO1_INTN1_IPP & GPIO2_INTN2_IPP; // interrupt pins are push-pull, active low
		MC3479Class::SPI_writeRegister(MC3479_GPIO_CTRL, data);

		// Register 0x40 (Tilt/Flip threshold LSB)
		data = 0xFF & 0x0F; // 15/255 Tilt/Flip Threshold LSB
		MC3479Class::SPI_writeRegister(MC3479_TF_THRESH_LSB, data);

		// Register 0x41 (Tilt/Flip threshold MSB)
		data = 0xFF & 0x00; // 0/255 Tilt/Flip Threshold MSB
		MC3479Class::SPI_writeRegister(MC3479_TF_THRESH_MSB, data);

		// Register 0x42 (Tilt/Flip De-bounce)
		data = 0xFF & 0x0F; // Tilt/Flip de-bounce duration to 15/255 before triggering IRQ
		MC3479Class::SPI_writeRegister(MC3479_TF_DB, data);

		// Register 0x43 (AnyMotion Threshold LSB)
		data = 0xFF & 0x0F; // 15/255 AnyMotionThreshold LSB
		MC3479Class::SPI_writeRegister(MC3479_AM_THRESH_LSB, data);

		// Register 0x44 (AnyMotion Threshold MSB)
		data = 0xFF & 0x00; // 0/255 AnyMotionThreshold MSB
		MC3479Class::SPI_writeRegister(MC3479_AM_THRESH_MSB, data);

		// Register 0x45 (AnyMotion De-bounce)
		data = 0xFF & 0x0F; // AnyMotion de-bounce duration to 15/255 before triggering IRQ
		MC3479Class::SPI_writeRegister(MC3479_AM_DB, data);

		// Register 0x46 (Shake Threshold LSB)
		data = 0xFF & 0x0F; // 15/255 Shake Threshold LSB
		MC3479Class::SPI_writeRegister(MC3479_SHK_THRESH_LSB, data);

		// Register 0x47 (Shake Threshold MSB)
		data = 0xFF & 0x00; // 0/255 Shake Threshold MSB
		MC3479Class::SPI_writeRegister(MC3479_SHK_THRESH_MSB, data);

		// Register 0x48 (Peak-to-Peak Duration LSB)
		data = 0xFF & 0x00; // Minimum duration to trigger a shake interrupt - unused???
		//TODO: are we using p2p or shake counter thresholds for interrupts??
		MC3479Class::SPI_writeRegister(MC3479_PK_P2P_DUR_THRESH_LSB, data);

		// Register 0x49 (Shake/Peak-to-Peak Duration MSB)
		data = 0xFF & 0x00; // Minimum duration to trigger a shake interrupt - unused???
		//TODO: are we using p2p or shake counter thresholds for interrupts??
		MC3479Class::SPI_writeRegister(MC3479_PK_P2P_DUR_THRESH_MSB, data);

		// Register 0x4A (Timer control)
		data = 0xFF & 0x00; //Tilt-35 latch disabled,tilt-35 angle detection duration = 1.6s (default)
		//TODO: if Tilt35 duration is too large for our control system, can we ignore configuration?
		MC3479Class::SPI_writeRegister(MC3479_TIMER_CTRL, data);

		// Register 0x4B (Read Count Register)
		data = 0xFF & 0x06; // default 6 reads when register 0x30 bit 7(FIFO_BURST) is enabled
		MC3479Class::SPI_writeRegister(MC3479_RD_CNT, data);



#else
		//Configuration using I2C:

		// Register 0x06 (interrupt enable)

		// Register 0x07 (MODE)

		// Register 0x08 (Sample  Rate)

		// Register 0x09 (Motion Control)

		// Register 0x20 (Range Select Control)

		// Register 0x21 (X-offset lSB)

		// Register 0x22 (X-offset MSB)

		// Register 0x23 (Y-offset LSB)

		// Register 0x24 (Y-offset MSB)

		// Register 0x25 (Z-offset LSB)

		// Register 0x26 (Z-offset MSB)

		// Register 0x27 (X Gain)

		// Register 0x28 (Y Gain)

		// Register 0x29 (Z Gain)

		// RegisteO 0x2D (FIFO Control)

		// Register 0x2E (FIFO Threshold)

		// Register 0x2F (FIFO Interrupt Status)

		// Register 0x30 (FIFO Control 2, Sample Rate 2)

		// Register 0x31 (Comm. Control)

		// Register 0x33 (GPIO Control)

		// Register 0x40 (Tilt/Flip threshold LSB)

		// Register 0x41 (Tilt/Flip threshold MSB)

		// Register 0x42 (Tilt/Flip Debounce)

		// Register 0x43 (AnyMotion Threshold LSB)

		// Register 0x44 (AnyMotion Threshold MSB)

		// Register 0x45 (AnyMotion Debounce)

		// Register 0x46 (Shake Threshold LSB)

		// Register 0x47 (Shake Threshold MSB)

		// Register 0x48 (Peak-to-Peak Duration LSB)

		// Register 0x49 (Shake/Peak-to-Peak Duration MSB)

		// Register 0x4A (Timer control)

		// Register 0x4B (Read Count Register)

#endif /* _SPI_COM_ENABLED */

	return;
}

// Set the accelerometer's sample rate from 50-2000Hz
bool MC3479Class::setSampleRate(uint8_t rate)
{
		uint8_t data = 0xFF & rate; // sample x,y,z @ 100Hz
		MC3479Class::SPI_writeRegister(MC3479_SR, data);
		return 1;
}

bool MC3479Class::getXYZ(uint8_t* xData, uint8_t* yData, uint8_t* zData)
{
	uint8_t Buffer[8];
	MC3479Class::burstSPI_readRegister(MC3479_XOUT_EX_L, &Buffer[0], 6);
	xData[0] = Buffer[0];
	xData[1] = Buffer[1];
	yData[0] = Buffer[2];
	yData[1] = Buffer[3];
	zData[0] = Buffer[4];
	zData[1] = Buffer[5];
	return 1;
}

uint8_t MC3479Class::getMotionFlagStatus()
{
	uint8_t status;
	MC3479Class::SPI_readRegister(MC3479_STATUS, &status);
	return status;
}

uint8_t MC3479Class::getMotionIrqStatus()
{
	uint8_t status;
	MC3479Class::SPI_readRegister(MC3479_INTR_STAT, &status);
	return status;
}

bool MC3479Class::clearMotionIrqStatus()
{
	uint8_t clear = 0x00;
	MC3479Class::SPI_writeRegister(MC3479_INTR_STAT, clear);
	return 1;
}

uint8_t MC3479Class::getFifoIrqStatus()
{
	uint8_t status;
	MC3479Class::SPI_readRegister(MC3479_FIFO_STAT, &status);
	return status;
}






