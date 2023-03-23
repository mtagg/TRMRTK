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





class MP6543HClass
{
public:
/*
 * Public Variables
 */
//TODO: Finalize member variables


//	GPIO_TypeDef * _x_DIR_GPIO;
//	GPIO_TypeDef * _x_nBRAKE_GPIO;
	GPIO_TypeDef *_x_nSLEEP_GPIO;
	GPIO_TypeDef *_x_nFAULT_GPIO;
	GPIO_TypeDef* _x_ENA;
	GPIO_TypeDef* _x_ENB;
	GPIO_TypeDef* _x_ENC;
//	uint16_t _x_DIR_PIN;
//	uint16_t _x_nBRAKE_PIN;
	uint16_t _x_nSLEEP_PIN;
	uint16_t _x_nFAULT_PIN;
	uint16_t _x_ENA_Pin;
	uint16_t _x_ENB_Pin;
	uint16_t _x_ENC_Pin;

	uint16_t _x_PWM_CHANNEL_A;
	uint16_t _x_PWM_CHANNEL_B;
	uint16_t _x_PWM_CHANNEL_C;
	TIM_HandleTypeDef * _x_PWM;

//	GPIO_TypeDef * _y_DIR_GPIO;
//	GPIO_TypeDef * _y_nBRAKE_GPIO;
//	GPIO_TypeDef * _y_nSLEEP_GPIO;
//	GPIO_TypeDef * _y_nFAULT_GPIO;
//	TIM_HandleTypeDef * _y_PWM_TIMER;
//	uint16_t _y_PWM_CHANNEL;
//	uint16_t _y_DIR_PIN;
//	uint16_t _y_nBRAKE_PIN;
//	uint16_t _y_nSLEEP_PIN;
//	uint16_t _y_nFAULT_PIN;



/*
 * MP6543HGL-B-Z Function-Headers
 */


//	bool setMotorPwm(uint8_t pwm);


	//x-axis motor functions:
	bool x_configMotorController(uint16_t xPwmChannelA, uint16_t xPwmChannelB,
									uint16_t xPwmChannelC, TIM_HandleTypeDef * xPwmTimer,
									GPIO_TypeDef* ENA, uint16_t ENA_Pin,
									GPIO_TypeDef* ENB, uint16_t ENB_Pin,
									GPIO_TypeDef* ENC, uint16_t ENC_Pin,
									GPIO_TypeDef* xSleep, uint16_t xSleepPin,
									GPIO_TypeDef* xFault, uint16_t xFaultPin);
	bool x_motorSleep();
	bool x_motorWake();
	bool x_motorFault();




};
//Initialize a class to be used whenever this library is used (allows for easier referencing, similar to Arduino libraries)
extern MP6543HClass MP6543H;


#endif /* SRC_MP6543B_H_ */
