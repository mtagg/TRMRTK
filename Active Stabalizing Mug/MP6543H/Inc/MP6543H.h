/*
 * MP6543A.h
 *
 *  Created on: Oct 9, 2022
 *      Author: mmmta
 *
 *      Header File for the MP6543A (Monolithic Power Systems.Inc) Motor Controller Driver
 *
 *      To avoid large sections of comments, functions/methods will use descriptive names.
 *      Any code that references the data-sheet will have the page number of the datasheet provided:
 *      https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP6543AGL/document_id/9083/
 */

#ifndef SRC_MP6543H_H
#define SRC_MP6543H_H

//Include HAL to allow passed-by-reference HAL objects
#include "stm32f1xx_hal.h"
//#include "main.h"

//#define X_MOTOR_PWM
//#define Y_MOTOR_PWM
//#define X_MOTOR_DIR
//#define Y_MOTOR_DIR
//#define X_MOTOR_BRAKE MP6543B_nBRAKE_X_Pin
//#define Y_MOTOR_BRAKE



class MP6543HClass
{
public:
/*
 * Public Variables
 */

	GPIO_TypeDef * _x_DIR_GPIO;
	//GPIO_TypeDef * _x_PWM_GPIO;
	GPIO_TypeDef * _x_nBRAKE_GPIO;
	GPIO_TypeDef *_x_nSLEEP_GPIO;
	GPIO_TypeDef *_x_nFAULT_GPIO;
	uint8_t _x_DIR_PIN;
	//uint8_t _x_PWM_PIN;
	uint8_t _x_nBRAKE_PIN;
	uint8_t _x_nSLEEP_PIN;
	uint8_t _x_nFAULT_PIN;

	GPIO_TypeDef * _y_DIR_GPIO;
	//uint8_t _y_PWM_GPIO;
	GPIO_TypeDef * _y_nBRAKE_GPIO;
	GPIO_TypeDef * _y_nSLEEP_GPIO;
	GPIO_TypeDef * _y_nFAULT_GPIO;
	uint8_t _y_DIR_PIN;
	//uint8_t _y_PWM_PIN;
	uint8_t _y_nBRAKE_PIN;
	uint8_t _y_nSLEEP_PIN;
	uint8_t _y_nFAULT_PIN;



/*
 * MP6543HGL-B-Z Function-Headers
 */
	//x axis headers:
	bool x_configMotorController();
	bool x_motorSleep();
	bool x_motorWake();
	bool x_setDir(bool forward_polarity);
	bool x_setPwm();
	bool x_motorBrake();
	bool x_motorFault();

	bool y_configMotorController();
	bool y_motorSleep();
	bool y_motorWake();
	bool y_setDir(bool fwd);
	bool y_setPwm();
	bool y_motorBrake();
	bool y_motorFault();


};
//Initialize a class to be used whenever this library is used (allows for easier referencing, similar to Arduino libraries)
extern MP6543HClass MP6543H;


#endif /* SRC_MP6543B_H_ */
