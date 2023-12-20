/*
 * control_signal_postprocessing.c
 *
 *  Created on: Dec 19, 2023
 *      Author: marcinpaluch
 */

#include "hardware_bridge.h"

int MOTOR_DYNAMICS_CORRECTED = true;
float MOTOR_CORRECTION[3] = {4898.18, 168.09, 123.46};

int control_signal_to_motor_command(float Q, float positionD) {
	float actualMotorCmd_float = 0.0;
    int actualMotorCmd = 0;

    if (MOTOR_DYNAMICS_CORRECTED) {

    	actualMotorCmd_float = Q * MOTOR_CORRECTION[0];
        if (actualMotorCmd != 0) {
            if (positionD > 0) {
            	actualMotorCmd_float += MOTOR_CORRECTION[1];
            } else if (positionD < 0) {
            	actualMotorCmd_float -= MOTOR_CORRECTION[2];
            }
        }
    } else {
    	actualMotorCmd_float = Q * MOTOR_FULL_SCALE;
    }

    actualMotorCmd = (int)actualMotorCmd_float;

    return actualMotorCmd;
}

void motor_command_safety_check(int* motor_command_ptr){
    // Check if motor power in safe boundaries
    if (*motor_command_ptr > MOTOR_FULL_SCALE_SAFE) *motor_command_ptr = MOTOR_FULL_SCALE_SAFE;
    else if (*motor_command_ptr < -MOTOR_FULL_SCALE_SAFE) *motor_command_ptr = -MOTOR_FULL_SCALE_SAFE;
}


void safety_switch_off(int* motor_command_ptr, int positionLimitLeft, int positionLimitRight){
	int position = Encoder_Read();
	// Disable motor if falls hard on either limit
	if ((*motor_command_ptr < 0) && (position < (positionLimitLeft + 20)))
	{
		*motor_command_ptr = 0;
	}
	else if ((*motor_command_ptr > 0) && (position > (positionLimitRight - 20)))
	{
		*motor_command_ptr = 0;
	}
}
