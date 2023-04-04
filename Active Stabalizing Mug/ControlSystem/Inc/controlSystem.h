/*
 * controlSystem.h
 *
 *  Created on: Nov 10, 2022
 *      Author: mmmta
 *
 *      This file contains any logic around the control system, such as PID controls, normalizing and interpretting data.
 *      Many of the variables and decisions made within the ControlClass class will then be used to interface the MP6543H and MC3479 hardware.
 *
 */
#ifndef SRC_CONTROLSYSTEM_H_
#define SRC_CONTROLSYSTEM_H_
//Include HAL to allow passed-by-reference HAL objects
#include "stm32f1xx_hal.h"
#include "math.h"
#include <queue>

#define THETA_MOVING_AVG_PERIOD 5
#define CTRL_LOOP_PERIOD  (double)0.001
#define N_SINE_IDX (int)360

using namespace std;


class ControlClass{
public:
/*
 * Class Variables
 */
	int8_t x_allowableAngle = 10;
	int16_t x_nominalAngle = 0;
	double RADS_TO_DEGREES = 180/3.14159;



uint8_t sineWave[N_SINE_IDX] = {
		127, 129, 131, 134, 136, 138, 140, 143, 145, 147,
		149, 151, 154, 156, 158, 160, 162, 164, 166, 169,
		171, 173, 175, 177, 179, 181, 183, 185, 187, 189,
		191, 193, 195, 196, 198, 200, 202, 204, 205, 207,
		209, 211, 212, 214, 216, 217, 219, 220, 222, 223,
		225, 226, 227, 229, 230, 231, 233, 234, 235, 236,
		237, 239, 240, 241, 242, 243, 243, 244, 245, 246,
		247, 248, 248, 249, 250, 250, 251, 251, 252, 252,
		253, 253, 253, 254, 254, 254, 254, 254, 254, 254,
		255, 254, 254, 254, 254, 254, 254, 254, 253, 253,
		253, 252, 252, 251, 251, 250, 250, 249, 248, 248,
		247, 246, 245, 244, 243, 243, 242, 241, 240, 239,
		237, 236, 235, 234, 233, 231, 230, 229, 227, 226,
		225, 223, 222, 220, 219, 217, 216, 214, 212, 211,
		209, 207, 205, 204, 202, 200, 198, 196, 195, 193,
		191, 189, 187, 185, 183, 181, 179, 177, 175, 173,
		171, 169, 166, 164, 162, 160, 158, 156, 154, 151,
		149, 147, 145, 143, 140, 138, 136, 134, 131, 129,
		127, 125, 123, 120, 118, 116, 114, 111, 109, 107,
		105, 103, 100, 98, 96, 94, 92, 90, 88, 85,
		83, 81, 79, 77, 75, 73, 71, 69, 67, 65,
		63, 61, 59, 58, 56, 54, 52, 50, 49, 47,
		45, 43, 42, 40, 38, 37, 35, 34, 32, 31,
		29, 28, 27, 25, 24, 23, 21, 20, 19, 18,
		17, 15, 14, 13, 12, 11, 11, 10, 9, 8,
		7, 6, 6, 5, 4, 4, 3, 3, 2, 2,
		1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
		1, 2, 2, 3, 3, 4, 4, 5, 6, 6,
		7, 8, 9, 10, 11, 11, 12, 13, 14, 15,
		17, 18, 19, 20, 21, 23, 24, 25, 27, 28,
		29, 31, 32, 34, 35, 37, 38, 40, 42, 43,
		45, 47, 49, 50, 52, 54, 56, 58, 59, 61,
		63, 65, 67, 69, 71, 73, 75, 77, 79, 81,
		83, 85, 88, 90, 92, 94, 96, 98, 100, 103,
		105, 107, 109, 111, 114, 116, 118, 120, 123, 125
};
/*
 * Kalman-Filter Variables
 */

	double qAngle= 0.002;	// uncertainty in the angle estimation process within the filter.
	double qBias = 0.002;	// uncertainty in the bias process within the filter.
	double rMeasure = 0.015; // represents the level of uncertainty in sensor measurements.
	double angle = 0;		// current angle, updated each time by the kalman filter
	double bias = 0;		// current bias, updated each time by kalman filter
	double P[2][2] = {{0,0},{0,0}}; //Covariance matrix
	double K[2] = {0,0};			// Kalman Gain Matrix
//	int16_t x_lastTenAngles[10] = {0};
	queue<double> x_previousAngles;
	queue<int16_t> x_previousEncoderPositions;

/*
 * PID Control Variables
 */
	double x_kp = 0.30;
	double x_ki = 0.30;
	double x_kd = 0.10;
	double x_P = 0;
	double x_I = 0;
	double x_D = 0;
	double tau = 0.01; // time constant x_D. 10ms to reach 2/3 of response. Lower Tau -> better LPF
	double error = 0;
	double previous_error = 0;
	double previous_angle = 0;

/*
 * ControlClass Function-Headers:
 */
	// takes a double representing arctan(theta) and
	// returns signed 16bit theta value from -90 to + 90
	double lookupAngle(int16_t z_acc, int16_t axis_acc);
	// Wrapper
	// Returns theta as the arctan of the ratio between data/z with angles between -90 and +90 degrees
	// where data is x or y accelerometer data values
	double x_normalizeTheta(uint8_t data0, uint8_t data1, uint8_t z0, uint8_t z1);

	bool initControlSystem();
//	uint8_t x_needCorrection(int16_t theta);
//	uint8_t y_needCorrection(int16_t theta);
	double averageAngle(queue<double> angles, int n);
	double kalmanFilter(double angleIn, double dTime);
	double calculateDelta(queue<double> angles);
	int16_t calculateDeltaEncoder(int16_t new_data);
	int16_t getPID(double angle, int16_t desired_angle);
	bool resetPID();
};



extern ControlClass ControlSystem;
#endif /* SRC_CONTROLSYSTEM_H_ */
