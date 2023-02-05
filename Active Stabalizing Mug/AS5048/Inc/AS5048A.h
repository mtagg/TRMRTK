#ifndef SRC_AS5048A_H
#define SRC_AS5048A_H

#include "stm32f1xx_hal.h"
#include "math.h"

//#define AS5048A_NOP 	(uint16_t) 0x0000
//#define AS5048A_R_ANGLE (uint16_t) 0xFFFF
//#define AS5048A_R_MAG 	(uint16_t) 0x7FFE
//#define AS5048A_R_ERROR	(uint16_t) 0x8001
#define AS5048A_SPI_TIMEOUT 10 //ms


class AS5048A_Class{
public:
	SPI_HandleTypeDef  * _SPI2;
	GPIO_TypeDef 	   * _CSN_GPIO;
	uint16_t _CSN_PIN;
	uint16_t Angle_Offset;
	uint8_t NOP [2] 	= {0x00,0x00};
	uint8_t ANGLE [2] 	= {0xFF,0xFF};
	uint8_t MAG [2]	= {0xFE,0x7F};
	uint8_t ERROR [2]	= {0x01,0x40};

	//Encoder Functions:
	bool SPI_Init(SPI_HandleTypeDef* spi, GPIO_TypeDef* csn_gpio,uint16_t csn_pin);
	bool send_Nop();
	uint16_t Transcieve_Nop();
	uint16_t readMagnitude();
	uint16_t readAngle();
	uint16_t readError();
};
extern AS5048A_Class AS5048A;
#endif //SRC_AS5048A_H
