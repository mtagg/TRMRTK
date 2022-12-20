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

#define MOTOR_CONTROL_DURATION 10 // in ms

/* CW and CCW are defined as the direction of the mug when facing the axis' motors from the outside of the mug, looking in.
 * so if the mug were to turn CCW on the handle's x-axis, this indicates a positive theta value.
 * To correct for this, we would set motor direction to CW to subtract from theta.
 * */
#define CLOCKWISE_DIR 1
#define COUNTER_CLOCKWISE_DIR !CLOCKWISE_DIR


class MP6543HClass
{
public:
/*
 * Public Variables
 */
//TODO: Finalize member variables


	GPIO_TypeDef * _x_DIR_GPIO;
	GPIO_TypeDef * _x_nBRAKE_GPIO;
	GPIO_TypeDef *_x_nSLEEP_GPIO;
	GPIO_TypeDef *_x_nFAULT_GPIO;
	TIM_HandleTypeDef * _x_PWM_TIMER;
	uint16_t _x_PWM_CHANNEL;
	uint16_t _x_DIR_PIN;
	uint16_t _x_nBRAKE_PIN;
	uint16_t _x_nSLEEP_PIN;
	uint16_t _x_nFAULT_PIN;

	GPIO_TypeDef * _y_DIR_GPIO;
	GPIO_TypeDef * _y_nBRAKE_GPIO;
	GPIO_TypeDef * _y_nSLEEP_GPIO;
	GPIO_TypeDef * _y_nFAULT_GPIO;
	TIM_HandleTypeDef * _y_PWM_TIMER;
	uint16_t _y_PWM_CHANNEL;
	uint16_t _y_DIR_PIN;
	uint16_t _y_nBRAKE_PIN;
	uint16_t _y_nSLEEP_PIN;
	uint16_t _y_nFAULT_PIN;



/*
 * MP6543HGL-B-Z Function-Headers
 */


	bool setMotorPwm(uint8_t pwm);


	//x-axis motor functions:
	bool x_configMotorController(uint16_t xPwmChannel, TIM_HandleTypeDef * pwmTimer,
									GPIO_TypeDef* xDir, uint16_t xDirPin,
										GPIO_TypeDef* xBrake, uint16_t xBrakePin,
											GPIO_TypeDef* xSleep, uint16_t xSleepPin,
												GPIO_TypeDef* xFault, uint16_t xFaultPin);

	bool x_motorSleep();
	bool x_motorWake();
	bool x_setMotorDir(bool forward_polarity);
	bool x_motorBrake(bool want_brake);
	bool x_startMotorPwmDuration(uint32_t duration);
	bool x_motorFault();

	//y-axis motor functions:
	bool y_configMotorController(uint16_t yPwmChannel, TIM_HandleTypeDef * pwmTimer,
									GPIO_TypeDef* yDir, uint16_t yDirPin,
										GPIO_TypeDef* yBrake, uint16_t yBrakePin,
											GPIO_TypeDef* ySleep, uint16_t ySleepPin,
												GPIO_TypeDef* yFault, uint16_t yFaultPin);

	bool y_motorSleep();
	bool y_motorWake();
	bool y_setMotorDir(bool forward_polarity);
	bool y_motorBrake(bool want_brake);
	bool y_startMotorPwmDuration(uint32_t duration);
	bool y_motorFault();


};
//Initialize a class to be used whenever this library is used (allows for easier referencing, similar to Arduino libraries)
extern MP6543HClass MP6543H;


#endif /* SRC_MP6543B_H_ */
