/*
 * MP6543A.cpp
 *
 *  Created on: Oct 9, 2022
 *      Author: mmmta
 */

#include "MP6543H.h"

MP6543HClass MP6543H;

bool MP6543HClass::setMotorPwm(uint8_t duty)
{
	if (duty  <= 100){
		TIM1->CCR1 = duty;
		return 1;
	}else{
		TIM1->CCR1 = 0;
		return 0;
	}
}

bool MP6543HClass::x_configMotorController(uint16_t xPwmChannel, TIM_HandleTypeDef * pwmTimer,
		GPIO_TypeDef* xDir, uint16_t xDirPin,
			GPIO_TypeDef* xBrake, uint16_t xBrakePin,
				GPIO_TypeDef* xSleep, uint16_t xSleepPin,
					GPIO_TypeDef* xFault, uint16_t xFaultPin)
{
	this->_x_PWM_CHANNEL = xPwmChannel;
	this->_x_PWM_TIMER = pwmTimer;
	this->_x_DIR_GPIO = xDir;
	this->_x_DIR_PIN = xDirPin;
	HAL_GPIO_WritePin(_x_DIR_GPIO, _x_DIR_PIN, GPIO_PIN_SET);
	this->_x_nBRAKE_GPIO = xBrake;
	this->_x_nBRAKE_PIN = xBrakePin;
	HAL_GPIO_WritePin(_x_nBRAKE_GPIO, _x_nBRAKE_PIN, GPIO_PIN_SET);
	this->_x_nSLEEP_GPIO = xSleep;
	this->_x_nSLEEP_PIN = xSleepPin;
	HAL_GPIO_WritePin(_x_nSLEEP_GPIO, _x_nSLEEP_PIN, GPIO_PIN_SET);
	this->_x_nFAULT_GPIO = xFault;
	this->_x_nFAULT_PIN = xFaultPin;

	return 1;
}

bool MP6543HClass::x_motorSleep()
{
	HAL_GPIO_WritePin(_x_nSLEEP_GPIO, _x_nSLEEP_PIN, GPIO_PIN_RESET);
	return 1;
}
bool MP6543HClass::x_motorWake()
{
	HAL_GPIO_WritePin(_x_nSLEEP_GPIO, _x_nSLEEP_PIN, GPIO_PIN_SET);
	HAL_Delay(1500);
	return 1;
}
bool MP6543HClass::x_setMotorDir(bool forward_polarity)
{
	if (forward_polarity){
		HAL_GPIO_WritePin(_x_DIR_GPIO, _x_DIR_PIN, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(_x_DIR_GPIO, _x_DIR_PIN, GPIO_PIN_RESET);
	}
	return 1;
}

bool MP6543HClass::x_motorBrake(bool want_brake)
{
	if (want_brake){
		HAL_GPIO_WritePin(_x_nBRAKE_GPIO, _x_nBRAKE_PIN, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(_x_nBRAKE_GPIO, _x_nBRAKE_PIN, GPIO_PIN_SET);
	}
	return 1;
}

bool MP6543HClass::x_startMotorPwmDuration(uint32_t duration)
{
	if (TIM1->CCR1 == 0){
		return 0;
	}
	if (duration > 10){
		duration = 10;
	}
	MP6543H.x_motorBrake(false);
	HAL_TIM_PWM_Start(_x_PWM_TIMER, _x_PWM_CHANNEL);
	HAL_Delay(duration);
	HAL_TIM_PWM_Stop(_x_PWM_TIMER, _x_PWM_CHANNEL);
	MP6543H.y_motorBrake(true);
	return 1;
}

bool MP6543HClass::x_motorFault()
{
	return !HAL_GPIO_ReadPin(_x_nFAULT_GPIO, _x_nFAULT_PIN);
}

bool MP6543HClass::y_configMotorController(uint16_t yPwmChannel, TIM_HandleTypeDef * pwmTimer,
		GPIO_TypeDef* yDir, uint16_t yDirPin,
			GPIO_TypeDef* yBrake, uint16_t yBrakePin,
				GPIO_TypeDef* ySleep, uint16_t ySleepPin,
					GPIO_TypeDef* yFault, uint16_t yFaultPin)
{
	this->_y_PWM_CHANNEL = yPwmChannel;
	this->_y_PWM_TIMER = pwmTimer;
	this->_y_DIR_GPIO = yDir;
	this->_y_DIR_PIN = yDirPin;
	HAL_GPIO_WritePin(_y_DIR_GPIO, _y_DIR_PIN, GPIO_PIN_SET);
	this->_y_nBRAKE_GPIO = yBrake;
	this->_y_nBRAKE_PIN = yBrakePin;
	HAL_GPIO_WritePin(_y_nBRAKE_GPIO, _y_nBRAKE_PIN, GPIO_PIN_SET);
	this->_y_nSLEEP_GPIO = ySleep;
	this->_y_nSLEEP_PIN = ySleepPin;
	HAL_GPIO_WritePin(_y_nSLEEP_GPIO, _y_nSLEEP_PIN, GPIO_PIN_SET);
	this->_y_nFAULT_GPIO = yFault;
	this->_y_nFAULT_PIN = yFaultPin;

	return 1;
}

bool MP6543HClass::y_motorSleep()
{
	HAL_GPIO_WritePin(_y_nSLEEP_GPIO, _y_nSLEEP_PIN, GPIO_PIN_RESET);
	return 1;
}

bool MP6543HClass::y_motorWake()
{
	HAL_GPIO_WritePin(_y_nSLEEP_GPIO, _y_nSLEEP_PIN, GPIO_PIN_SET);
	HAL_Delay(1500);
	return 1;
}

bool MP6543HClass::y_setMotorDir(bool forward_polarity)
{
	if (forward_polarity){
		HAL_GPIO_WritePin(_y_DIR_GPIO, _y_DIR_PIN, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(_y_DIR_GPIO, _y_DIR_PIN, GPIO_PIN_RESET);
	}
	return 1;
}


bool MP6543HClass::y_motorBrake(bool want_brake)
{
	if (want_brake){
		HAL_GPIO_WritePin(_y_nBRAKE_GPIO, _y_nBRAKE_PIN, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(_y_nBRAKE_GPIO, _y_nBRAKE_PIN, GPIO_PIN_SET);
	}
	return 1;
}

bool MP6543HClass::y_startMotorPwmDuration(uint32_t duration)
{
	if (TIM1->CCR1 == 0){
			return 0;
	}
	if (duration > 10){
		duration = 10;
	}
	MP6543H.y_motorBrake(false);
	HAL_TIM_PWM_Start(_y_PWM_TIMER, _y_PWM_CHANNEL);
	HAL_Delay(duration);
	HAL_TIM_PWM_Stop(_y_PWM_TIMER, _y_PWM_CHANNEL);
	MP6543H.y_motorBrake(true);
	return 1;
}

bool MP6543HClass::y_motorFault()
{
	return !HAL_GPIO_ReadPin(_y_nFAULT_GPIO, _y_nFAULT_PIN);
}
