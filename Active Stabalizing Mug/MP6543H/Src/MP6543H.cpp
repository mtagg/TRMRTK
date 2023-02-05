/*
 * MP6543A.cpp
 *
 *  Created on: Oct 9, 2022
 *      Author: mmmta
 */

#include "MP6543H.h"

MP6543HClass MP6543H;

//bool MP6543HClass::setMotorPwm(uint8_t duty)
//{
//	if (duty  <= 100){
//		TIM1->CCR1 = duty;
//		return 1;
//	}else{
//		TIM1->CCR1 = 0;
//		return 0;
//	}
//}

bool MP6543HClass::x_configMotorController(uint16_t xPwmChannelA, uint16_t xPwmChannelB,
											uint16_t xPwmChannelC, TIM_HandleTypeDef * xPwmTimer,
											GPIO_TypeDef* ENA, uint16_t ENA_Pin,
											GPIO_TypeDef* ENB, uint16_t ENB_Pin,
											GPIO_TypeDef* ENC, uint16_t ENC_Pin,
											GPIO_TypeDef* xSleep, uint16_t xSleepPin,
											GPIO_TypeDef* xFault, uint16_t xFaultPin){
	this->_x_PWM = xPwmTimer;
	this->_x_PWM_CHANNEL_A = xPwmChannelA;
	this->_x_PWM_CHANNEL_B = xPwmChannelB;
	this->_x_PWM_CHANNEL_C = xPwmChannelC;
	this->_x_ENA = ENA;
	this->_x_ENB = ENB;
	this->_x_ENC = ENC;
	this->_x_ENA_Pin = ENA_Pin;
	this->_x_ENB_Pin = ENB_Pin;
	this->_x_ENC_Pin = ENC_Pin;


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
//bool MP6543HClass::x_setMotorDir(bool forward_polarity)
//{
//	if (forward_polarity){
//		HAL_GPIO_WritePin(_x_DIR_GPIO, _x_DIR_PIN, GPIO_PIN_SET);
//	}
//	else{
//		HAL_GPIO_WritePin(_x_DIR_GPIO, _x_DIR_PIN, GPIO_PIN_RESET);
//	}
//	return 1;
//}
//
//bool MP6543HClass::x_motorBrake(bool want_brake)
//{
//	if (want_brake){
//		HAL_GPIO_WritePin(_x_nBRAKE_GPIO, _x_nBRAKE_PIN, GPIO_PIN_RESET);
//	}else{
//		HAL_GPIO_WritePin(_x_nBRAKE_GPIO, _x_nBRAKE_PIN, GPIO_PIN_SET);
//	}
//	return 1;
//}
//
//bool MP6543HClass::x_startMotorPwmDuration(uint32_t duration)
//{
//	if (TIM1->CCR1 == 0){
//		return 0;
//	}
//	if (duration > 10){
//		duration = 10;
//	}
//	MP6543H.x_motorBrake(false);
//	HAL_TIM_PWM_Start(_x_PWM_TIMER, _x_PWM_CHANNEL);
//	HAL_Delay(duration);
//	HAL_TIM_PWM_Stop(_x_PWM_TIMER, _x_PWM_CHANNEL);
//	MP6543H.y_motorBrake(true);
//	return 1;
//}

bool MP6543HClass::x_motorFault()
{
	return !HAL_GPIO_ReadPin(_x_nFAULT_GPIO, _x_nFAULT_PIN);
}

//bool MP6543HClass::y_configMotorController(uint16_t yPwmChannel, TIM_HandleTypeDef * pwmTimer,
//		GPIO_TypeDef* yDir, uint16_t yDirPin,
//			GPIO_TypeDef* yBrake, uint16_t yBrakePin,
//				GPIO_TypeDef* ySleep, uint16_t ySleepPin,
//					GPIO_TypeDef* yFault, uint16_t yFaultPin)
//{
//	this->_y_PWM_CHANNEL = yPwmChannel;
//	this->_y_PWM_TIMER = pwmTimer;
//	this->_y_DIR_GPIO = yDir;
//	this->_y_DIR_PIN = yDirPin;
//	HAL_GPIO_WritePin(_y_DIR_GPIO, _y_DIR_PIN, GPIO_PIN_SET);
//	this->_y_nBRAKE_GPIO = yBrake;
//	this->_y_nBRAKE_PIN = yBrakePin;
//	HAL_GPIO_WritePin(_y_nBRAKE_GPIO, _y_nBRAKE_PIN, GPIO_PIN_SET);
//	this->_y_nSLEEP_GPIO = ySleep;
//	this->_y_nSLEEP_PIN = ySleepPin;
//	HAL_GPIO_WritePin(_y_nSLEEP_GPIO, _y_nSLEEP_PIN, GPIO_PIN_SET);
//	this->_y_nFAULT_GPIO = yFault;
//	this->_y_nFAULT_PIN = yFaultPin;
//
//	return 1;
//}
//
//bool MP6543HClass::y_motorSleep()
//{
//	HAL_GPIO_WritePin(_y_nSLEEP_GPIO, _y_nSLEEP_PIN, GPIO_PIN_RESET);
//	return 1;
//}
//
//bool MP6543HClass::y_motorWake()
//{
//	HAL_GPIO_WritePin(_y_nSLEEP_GPIO, _y_nSLEEP_PIN, GPIO_PIN_SET);
//	HAL_Delay(1500);
//	return 1;
//}
//
//bool MP6543HClass::y_setMotorDir(bool forward_polarity)
//{
//	if (forward_polarity){
//		HAL_GPIO_WritePin(_y_DIR_GPIO, _y_DIR_PIN, GPIO_PIN_SET);
//	}
//	else{
//		HAL_GPIO_WritePin(_y_DIR_GPIO, _y_DIR_PIN, GPIO_PIN_RESET);
//	}
//	return 1;
//}
//
//
//bool MP6543HClass::y_motorBrake(bool want_brake)
//{
//	if (want_brake){
//		HAL_GPIO_WritePin(_y_nBRAKE_GPIO, _y_nBRAKE_PIN, GPIO_PIN_RESET);
//	}else{
//		HAL_GPIO_WritePin(_y_nBRAKE_GPIO, _y_nBRAKE_PIN, GPIO_PIN_SET);
//	}
//	return 1;
//}
//
//bool MP6543HClass::y_startMotorPwmDuration(uint32_t duration)
//{
//	if (TIM1->CCR1 == 0){
//			return 0;
//	}
//	if (duration > 10){
//		duration = 10;
//	}
//	MP6543H.y_motorBrake(false);
//	HAL_TIM_PWM_Start(_y_PWM_TIMER, _y_PWM_CHANNEL);
//	HAL_Delay(duration);
//	HAL_TIM_PWM_Stop(_y_PWM_TIMER, _y_PWM_CHANNEL);
//	MP6543H.y_motorBrake(true);
//	return 1;
//}
//
//bool MP6543HClass::y_motorFault()
//{
//	return !HAL_GPIO_ReadPin(_y_nFAULT_GPIO, _y_nFAULT_PIN);
//}
