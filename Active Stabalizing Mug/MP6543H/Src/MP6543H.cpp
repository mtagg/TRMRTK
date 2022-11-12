/*
 * MP6543A.cpp
 *
 *  Created on: Oct 9, 2022
 *      Author: mmmta
 */

#include "MP6543H.h"

MP6543HClass MP6543H;



bool MP6543HClass::x_configMotorController(uint16_t xPwmChannel, TIM_HandleTypeDef * pwmTimer,
		GPIO_TypeDef* xDir, uint16_t xDirPin,
			GPIO_TypeDef* xDirBrake, uint16_t xBrakePin,
				GPIO_TypeDef* xSleep, uint16_t xSleepPin,
					GPIO_TypeDef* xFault, uint16_t xFaultPin)
{
	return 1;
}


bool MP6543HClass::y_configMotorController(uint16_t yPwmChannel, TIM_HandleTypeDef * pwmTimer,
		GPIO_TypeDef* yDir, uint16_t yDirPin,
			GPIO_TypeDef* yDirBrake, uint16_t yBrakePin,
				GPIO_TypeDef* ySleep, uint16_t ySleepPin,
					GPIO_TypeDef* yFault, uint16_t yFaultPin)
{
	return 1;
}

