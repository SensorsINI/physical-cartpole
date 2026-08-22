/*
 * angle_processing.h
 *
 *  Created on: 12 Sep 2023
 *      Author: marcinpaluch
 */

#ifndef ANGLE_PROCESSING_H_
#define ANGLE_PROCESSING_H_

void set_timesteps_for_derivative(unsigned short timesteps);
void average_derivatives(float* angleDPtr, float* positionDPtr);
void process_angle(int angleSamples[], unsigned short angleSampIndex, unsigned short angle_averageLen, int* anglePtr, int* angle_raw_Ptr, float* angleDPtr, int* invalid_stepPtr);

// Hardware dead-zone detection (Zynq): call once per control poll, BEFORE
// process_angle(), with 1 if the FPGA filter block reported rail contact /
// window contamination since the previous poll, 0 otherwise. Once called,
// the derivative-jump heuristic in treat_deadangle_with_derivative() is
// replaced by this exact hardware signal. Never called on STM, where the
// heuristic remains active.
void report_hardware_deadzone(int contaminated);
int anomaly_detection(int* angleSamples, unsigned short angleSampIndex, unsigned short angle_averageLen);
void treat_deadangle_with_derivative(int* anglePtr, int invalid_step);
void calculate_position_difference_per_timestep(short* positionPtr, float* positionDPtr);

int wrapLocal(int angle);
float wrapLocal_float(float angle);
float wrapLocal_rad(float angle);
int unwrapLocal(int previous, int current);
int wrap(int current);
int unwrap(int previous, int current);


#endif /* ANGLE_PROCESSING_H_ */
