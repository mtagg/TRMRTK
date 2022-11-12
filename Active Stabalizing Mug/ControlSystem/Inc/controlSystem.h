/*
 * controlSystem.h
 *
 *  Created on: Nov 10, 2022
 *      Author: mmmta
 *
 */
#ifndef SRC_CONTROLSYSTEM_H_
#define SRC_CONTROLSYSTEM_H_
//Include HAL to allow passed-by-reference HAL objects
#include "stm32f1xx_hal.h"






class ControlClass{
public:
/*
 * Class Variables
 */

	// look-up table for determining theta based on :
	// arctan(theta) = z_acceleration/x/y_acceleration ratios
	// the index corresponding to a ratio is the angle in degrees from 0 to 89 degrees
	double angleTable [90] =
	{
			0.0, 0.0175, 0.0349, 0.0524, 0.0699, 0.0875, 0.1051, 0.1228, 0.1405, 0.1584, 		// 0-9 degrees
			0.1763, 0.1944, 0.2126, 0.2309, 0.2493, 0.2679, 0.2867, 0.3057, 0.3249, 0.3443, 	// 10-19
			0.364, 0.3839, 0.404, 0.4245, 0.4452, 0.4663, 0.4877, 0.5095, 0.5317, 0.5543, 		// 20-29
			0.5774, 0.6009, 0.6249, 0.6494, 0.6745, 0.7002, 0.7265, 0.7536, 0.7813, 0.8098, 	// 30-39
			0.8391, 0.8693, 0.9004, 0.9325, 0.9657, 1.0, 1.0355, 1.0724, 1.1106, 1.1504, 		// 40-49
			1.1918, 1.2349, 1.2799, 1.327, 1.3764, 1.4281, 1.4826, 1.5399, 1.6003, 1.6643, 		// 50-59
			1.7321, 1.804, 1.8807, 1.9626, 2.0503, 2.1445, 2.246, 2.3559, 2.4751, 2.6051, 		// 60-69
			2.7475, 2.9042, 3.0777, 3.2709, 3.4874, 3.7321, 4.0108, 4.3315, 4.7046, 5.1446, 	// 70-79
			5.6713, 6.3138, 7.1154, 8.1443, 9.5144, 11.4301, 14.3007, 19.0811, 28.6363, 57.29 	// 80-89 degrees

	};


/*
 * ControlClass Function-Headers:
 */

	// takes a double representing arctan(theta) and
	// returns signed 16bit theta value from -90 to + 90
	int16_t lookupAngle(int16_t z_acc, int16_t axis_acc);

	// Wrapper method
	// Returns theta as the arctan of the ratio between data/z with angles between -90 and +90 degrees
	// where data is x or y accelerometer data values
	int16_t normalizeTheta(uint8_t data0, uint8_t data1, uint8_t z0, uint8_t z1);

	bool x_needCorrection();
	bool y_needCorrection();

	bool x_setPwm();
	bool y_setPwm();


};

extern ControlClass ControlSystem;


#endif /* SRC_CONTROLSYSTEM_H_ */
