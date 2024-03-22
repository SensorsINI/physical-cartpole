/*
 * control_signal_posprocessing.h
 *
 *  Created on: Dec 19, 2023
 *      Author: marcinpaluch
 */

#ifndef CONTROL_SIGNAL_POSPROCESSING_H_
#define CONTROL_SIGNAL_POSPROCESSING_H_

int control_signal_to_motor_command(float Q, float positionD, bool correct_motor_dynamics);
void motor_command_safety_check(int* motor_command_ptr);
void safety_switch_off(int* motor_command_ptr, int positionLimitLeft, int positionLimitRight);

#endif /* CONTROL_SIGNAL_POSPROCESSING_H_ */
