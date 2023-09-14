/*
 * angle_processing.h
 *
 *  Created on: 12 Sep 2023
 *      Author: marcinpaluch
 */

#ifndef ANGLE_PROCESSING_H_
#define ANGLE_PROCESSING_H_

#define CONTROL_ANGLE_AVERAGE_LEN	16  			// Number of samples to average over to determine angular displacement (max is 32)
#define MAX_ADC_STEP 20								// If jump between two consecutive ADC measurements is bigger than this, the measurement counts as invalid


void process_angle(int angleSamples[], unsigned short angleSampIndex, unsigned short angle_averageLen, int* anglePtr, int* angleDPtr, int* invalid_stepPtr);
int anomaly_detection(int* angleSamples, unsigned short angleSampIndex, unsigned short angle_averageLen);
void treat_deadangle_with_derivative(int* anglePtr, int invalid_step);

int wrapLocal(int angle);
int unwrapLocal(int previous, int current);
int wrap(int current);
int unwrap(int previous, int current);


#endif /* ANGLE_PROCESSING_H_ */
