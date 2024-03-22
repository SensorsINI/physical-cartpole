/*
 * angle_processing.h
 *
 *  Created on: 12 Sep 2023
 *      Author: marcinpaluch
 */

#ifndef ANGLE_PROCESSING_H_
#define ANGLE_PROCESSING_H_

void average_derivatives(float* angleDPtr, float* positionDPtr);
void process_angle(int angleSamples[], unsigned short angleSampIndex, unsigned short angle_averageLen, int* anglePtr, float* angleDPtr, int* invalid_stepPtr);
int anomaly_detection(int* angleSamples, unsigned short angleSampIndex, unsigned short angle_averageLen);
void treat_deadangle_with_derivative(int* anglePtr, int invalid_step);
void calculate_position_difference_per_timestep(short* positionPtr, float* positionDPtr);

int wrapLocal(int angle);
float wrapLocal_rad(float angle);
int unwrapLocal(int previous, int current);
int wrap(int current);
int unwrap(int previous, int current);


#endif /* ANGLE_PROCESSING_H_ */
