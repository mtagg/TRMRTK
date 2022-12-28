/*
 * controlSystem.cpp
 *
 *  Created on: Nov 10, 2022
 *      Author: mmmta
 *
 */
#include "controlSystem.h"

ControlClass ControlSystem;

int16_t ControlClass::lookupAngle(int16_t z_acc, int16_t axis_acc)
{


	if (z_acc == 0){
		//divide by 0 case:
		return (axis_acc < 0) ? (int16_t) -90 : (int16_t) 90;
	}

	double arctan_theta = (double)((double)axis_acc/(double)z_acc);
	int sign;

	//convert ratio to positive for quicker LUT result
	if (arctan_theta < 0)
	{
		arctan_theta *= -1;
		sign = -1;
	}
	else
	{
		sign = 1;
	}
	uint8_t i;
	for (i = 0; i < sizeof(ControlSystem.angleTable)/sizeof(double); i++)
	{
		if (arctan_theta < ControlSystem.angleTable[i+1]){
			break;
		}
	}

	return sign * i; //angle between -89 and +89
}


int16_t ControlClass::normalizeTheta(uint8_t data0, uint8_t data1, uint8_t z0, uint8_t z1)
{
	int16_t z_acc = (z1 << 8) | z0;
	int16_t axis_acc = (data1 << 8) | data0;
	return lookupAngle(z_acc, axis_acc);
}

// TODO: Do we need to have an init method, or keep all variables in main???
bool ControlClass::updateControlSystem(int16_t x_nominal, int16_t y_nominal, int8_t x_allowable, int8_t y_allowable)
{
	this->x_nominalAngle = x_nominal;
	this->x_allowableAngle = x_allowable;
	this->y_nominalAngle = y_nominal;
	this->y_allowableAngle = y_allowable;
	return 1;
}

uint8_t ControlClass::x_needCorrection(int16_t theta)
{
	int16_t delta = theta - this->x_nominalAngle;
	if (delta < 0){
		delta *= -1;
	}
	if (delta > this->x_allowableAngle){
		return delta;
	}
	// no correction necessary
	return false;

}
uint8_t ControlClass::y_needCorrection(int16_t theta)
{
	int16_t delta = theta - this->y_nominalAngle;
	if (delta < 0){
		delta *= -1;
	}
	if (delta > this->y_allowableAngle){
		// correction necessary
		return delta;
	}
	// no correction necessary
	return 0;
}

uint8_t ControlClass::x_calcPwm(int16_t x_nominal, int16_t x_theta, int16_t y_nominal, int16_t y_theta)
{
	uint8_t delta = ControlClass::x_needCorrection(x_theta);
	if (delta > 45){
		return (uint8_t)100;
	}
	if (delta){
		return (uint8_t)(1 + 70*(delta/45));
	}
	return 0;
}
uint8_t ControlClass::y_calcPwm(int16_t x_nominal, int16_t x_theta, int16_t y_nominal, int16_t y_theta)
{
	uint8_t delta = ControlClass::y_needCorrection(y_theta);
	if (delta > 45){
		return (uint8_t)100;
	}
	if (delta){
		return (uint8_t)(1 + 70*(delta/45));
	}
	return 0;
}

