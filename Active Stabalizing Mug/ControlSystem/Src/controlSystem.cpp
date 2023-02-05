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


	// Check for divide by 0 at +/- 90 degrees
	if (z_acc == 0){
		return (axis_acc < 0) ? (int16_t) -90 : (int16_t) 90;
	}
	// Mug needs to remain between -90 and +90 degrees otherwise this method needs to be updated.
	// It can be argued that we have massively failed if the mug reaches +/- 90 degrees tho, haha.
	double theta = atan((double)axis_acc/(double)z_acc)*this->RADS_TO_DEGREES;

	// Return integer value of theta (rounded closer to 0)
	return (int16_t)theta;

}


int16_t ControlClass::normalizeTheta(uint8_t data0, uint8_t data1, uint8_t z0, uint8_t z1)
{
	int16_t z_acc = (z1 << 8) | (z0 & 0x00FF);
	int16_t axis_acc = (data1 << 8) | (data0 & 0x00FF);
	//TODO: need to split this into x and y components and update x and y queues seperately. Also calculate current average and return.


	return lookupAngle(z_acc, axis_acc);
}

// TODO: Do we need to keep nominal/allowable angles here? should this just be an initialization function?
bool ControlClass::initControlSystem(int16_t x_nominal, int8_t x_allowable)
{

	//TODO: implement x and y angle updates/averages
	for (int i = 0; i < 10; i++){
		this->x_previousAngles.push(0);
//		this->y_previousAngles.push(0);
	}


	this->x_nominalAngle = x_nominal;
	this->x_allowableAngle = x_allowable;

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


