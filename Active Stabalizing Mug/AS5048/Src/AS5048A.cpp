
#include "AS5048A.h"

AS5048A_Class AS5048A;

bool AS5048A_Class::SPI_Init(SPI_HandleTypeDef* spi, GPIO_TypeDef* csn_gpio,uint16_t csn_pin){
	this->_SPI2 = spi;
	this->_CSN_GPIO = csn_gpio;
	this->_CSN_PIN = csn_pin;
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
	this->errorCount = 0;
	return 1;
}
bool AS5048A_Class::send_Nop(){
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->_SPI2, &this->NOP[0], sizeof(this->NOP), AS5048A_SPI_TIMEOUT);
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
	return 1;
}
uint16_t AS5048A_Class::Transcieve_Nop(){
	uint8_t regValue [2];
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(this->_SPI2, &this->NOP[0], &regValue[0],  sizeof(this->NOP), AS5048A_SPI_TIMEOUT);
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
	return (uint16_t)(((regValue[0]<<8)&0xFF00) | regValue[1]);
}
//uint16_t AS5048A_Class::readMagnitude(){
//	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(this->_SPI2, this->MAG, sizeof(MAG), AS5048A_SPI_TIMEOUT);
//	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
//	return AS5048A.Transcieve_Nop() & 0x3FFF; //removes error/parity
//}
//uint16_t AS5048A_Class::readAngle(){
//	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(this->_SPI2, (uint8_t*)this->ANGLE, 1, AS5048A_SPI_TIMEOUT);
//	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
//	return AS5048A.Transcieve_Nop() & 0x3FFF; //removes error/parity
//}
uint16_t AS5048A_Class::readAngleSequential(){
	uint8_t regValue [2];
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(this->_SPI2, &this->ANGLE[0], &regValue[0], sizeof(this->ANGLE), AS5048A_SPI_TIMEOUT);
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
	// Check for error bit and re-call if found
	uint16_t regValue16 = (uint16_t)(((regValue[0]<<8)&0xFF00) | regValue[1]);
	if ((regValue16 & 0x4000) && this->errorCount < 5){
		// Error bit set, recursively read up to 5 times
		this->errorCount += 1;
		return AS5048A_Class::readAngleSequential();
	}
	this->errorCount = 0;
	return convertMagToDegrees(regValue16 & 0x3FFF); //removes error/parity
}
//uint16_t AS5048A_Class::readMagSequential(){
//	uint8_t regValue [2];
//	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(this->_SPI2, &this->MAG[0], &regValue[0], sizeof(this->MAG), AS5048A_SPI_TIMEOUT);
//	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
//	return convertMagToDegrees((uint16_t)(((regValue[0]<<8)&0xFF00) | regValue[1])&0x3FFF); //removes error/parity
//}
uint16_t AS5048A_Class::readError(){
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->_SPI2, &this->ERROR[0], sizeof(this->ERROR), AS5048A_SPI_TIMEOUT);
	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
	return AS5048A.Transcieve_Nop() & 0x07; //remove unused bits
}
//uint16_t AS5048A_Class::readZero6LSB(){
//	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(this->_SPI2, &this->ZERO6LSB[0], sizeof(this->ZERO6LSB), AS5048A_SPI_TIMEOUT);
//	HAL_GPIO_WritePin(this->_CSN_GPIO, this->_CSN_PIN, GPIO_PIN_SET);
//	return AS5048A.Transcieve_Nop(); //removes error/parity
//}

uint16_t AS5048A_Class::convertMagToDegrees(uint16_t mag){
	double fract = ((double)mag-MAG_MIN)/(MAG_MAX-MAG_MIN);
	return (uint16_t)(fract*359);
}
