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

	double arctan_theta = (double)(axis_acc/z_acc);
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
	return lookupAngle(axis_acc,z_acc);




}
