/*
 * controlSystem.cpp
 *
 *  Created on: Nov 10, 2022
 *      Author: mmmta
 *
 */
#include "controlSystem.h"

ControlClass ControlSystem;

double ControlClass::lookupAngle(int16_t z_acc, int16_t axis_acc)
{


	// Check for divide by 0 at +/- 90 degrees
	if (z_acc == 0){
		return (axis_acc < 0) ? (int16_t) -90 : (int16_t) 90;
	}
	// Mug needs to remain between -90 and +90 degrees otherwise this method needs to be updated.
	// It can be argued that we have massively failed if the mug reaches +/- 90 degrees tho, haha.
	double theta = atan((double)axis_acc/(double)z_acc)*this->RADS_TO_DEGREES;
	return theta;
}


double ControlClass::x_normalizeTheta(uint8_t data0, uint8_t data1, uint8_t z0, uint8_t z1)
{
	int16_t z_acc = (z1 << 8) | (z0 & 0x00FF);
	int16_t axis_acc = (data1 << 8) | (data0 & 0x00FF);
	double current_theta = lookupAngle(z_acc, axis_acc);
	double kalmanAngle = ControlClass::kalmanFilter(current_theta, CTRL_LOOP_PERIOD);
	this->x_previousAngles.pop();
	this->x_previousAngles.push(kalmanAngle);
//	double avgAngle =  ControlClass::averageAngle(this->x_previousAngles, this->x_previousAngles.size());
	return kalmanAngle;
}



bool ControlClass::initControlSystem()
{

	for (unsigned int i = 0; i < THETA_MOVING_AVG_PERIOD; i++){
		this->x_previousAngles.push(0);
		this->x_previousEncoderPositions.push(0);
//		this->y_previousAngles.push(0);
	}

	return 1;
}

//uint8_t ControlClass::x_needCorrection(int16_t theta)
//{
//	int16_t delta = theta - this->x_nominalAngle;
//	if (delta < 0){
//		delta *= -1;
//	}
//	if (delta > this->x_allowableAngle){
//		return delta;
//	}
//	// no correction necessary
//	return false;
//}
//
//


double ControlClass::averageAngle(queue<double> angles, int n){
	queue<double> copy(angles);
	double sum = 0;
	for (int i = 0; i < n; ++i){
		sum += copy.front(); //add first element to total
		copy.pop(); //remove first element
	}
	return (sum/n);
}

// Kalman-Filter:
// More info on the theory here: https://arxiv.org/ftp/arxiv/papers/1204/1204.0375.pdf
// Decent video that explains (with a part 2&3 for code examples): https://www.youtube.com/watch?v=TEKPcyBwEH8&list=PLvKAPIGzFEr8n7WRx8RptZmC1rXeTzYtA&index=2&ab_channel=CppMonk
double ControlClass::kalmanFilter(double angleIn, double dTime) {
// Typically with an IMU, we would have gyro AND accelerometer in this function, but this only accounts for accelly
  this->angle += dTime * (angleIn - this->bias); // Angle = previous angle plus (new angle - bias) * delta time.

// Update estimation error co-variance, ie. how much variance between angle measurements
  this->P[0][0] += dTime * (dTime*this->P[1][1] - this->P[0][1] - this->P[1][0] + this->qAngle);
  this->P[0][1] -= dTime * P[1][1];
  this->P[1][0] -= dTime * P[1][1];
  this->P[1][1] += this->qBias * dTime;

  // Calculate Kalman gain matrix
  double S = this->P[0][0] + this->rMeasure;
  this->K[0] = this->P[0][0] / S;
  this->K[1] = this->P[1][0] / S;

// Update angle and bias
  double y = angleIn - angle;		// y = difference between measured and
  angle += K[0] * y; 				// angle is a function of delta-time, uncertainty in the kalman filter, and the sensor angle vs estimation.
  bias += K[1] * y; 				// bias is a function of delta-time, previous bias, and the sensor angle vs estimation.

 // Update estimation error co-variance based on Kalman gain matrix
  double P00_temp = P[0][0];
  double P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return this->angle;
}

double ControlClass::calculateDelta(queue<double> angles){
	//TODO: update this to only consider the last 2 angles??
	auto front = angles.front();
	auto back = angles.back();
	return (double)(front - back);
}

int16_t ControlClass::calculateDeltaEncoder(int16_t new_data){
	// Update the Encoder Data queue:
	this->x_previousEncoderPositions.pop();
	this->x_previousEncoderPositions.push(new_data);
	// Calculate the difference between first and last value in queue:
	auto front = this->x_previousEncoderPositions.front();
	auto back = this->x_previousEncoderPositions.back();
	return (int16_t)(front-back);
}

int16_t ControlClass::getPID(double angle, int16_t desired_angle){
	this->error = angle-desired_angle;

	// Calculate Proportional:
	this->x_P = this->x_kp * this->error;

	// Calculate Integrator:
	this->x_I = this->x_I + 0.5*this->x_ki*CTRL_LOOP_PERIOD*(this->error + this->previous_error);

	// Calculate Derivator?:
	this->x_D = x_D*(2.0*this->tau - CTRL_LOOP_PERIOD)/(2.0*this->tau + CTRL_LOOP_PERIOD)
					+ 2.0*(this->x_kd)*(this->error - this->previous_error);
//	this->x_D = x_kd * (this->error - this->previous_error) / CTRL_LOOP_PERIOD;

	// Update PID value and 'previous' variables
	int16_t PID = (this->x_P + this->x_I + this-> x_D);
	this->previous_error = error;
	this->previous_angle = angle;
	return PID;
}

bool ControlClass::resetPID(){
	this->x_P = 0;
	this->x_I = 0;
	this->x_D = 0;
	return 1;
}

