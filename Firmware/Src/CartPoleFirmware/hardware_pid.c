/*
 * hardware_pid.c
 *
 *  Created on: 19 Dec 2023
 *      Author: marcinpaluch
 */

#include "hardware_bridge.h"
#include "communication_with_PC.h"
#include "hardware_pid.h"
#include <stdio.h>
#include <math.h>

float ANGLE_KP      =          18.0f;
float ANGLE_KI      =          38.0f	;
float ANGLE_KD      =          4.0f;

float POSITION_KP   =          22.0f;
float POSITION_KI   =          1.0f;
float POSITION_KD   =          12.0f;

float sensitivity_pP_gain = 1.0;
float sensitivity_pI_gain = 1.0;
float sensitivity_pD_gain = 0.01;

float sensitivity_aP_gain = 1.0;
float sensitivity_aI_gain = 1.0;
float sensitivity_aD_gain = 0.01;

// Error and previous error variables
float position_error, position_error_previous, position_error_diff = 0.0;
float position_error_integral = 0.0;
float angle_error, angle_error_previous, angle_error_diff = 0.0;
float angle_error_integral = 0.0;

// Last time variable
float time_last = -1.0;

// PID step function
float pid_step(float angle, float angleD, float position, float positionD, float target_position, float time) {

	float time_difference;
    float Q_position, Q_angle, Q;
    // Time difference calculation
    if (time_last < 0.0) {
        time_difference = 0.0;
    } else {
        time_difference = time - time_last;
    }

    if (time_difference > 0.1) {
        time_difference = 0.0;
    }

    time_last = time;

    // Position PID
    position_error = position - target_position;

    if (time_difference > 0.0001) {
        position_error_diff = (position_error - position_error_previous) / time_difference;
    } else {
        position_error_diff = 0.0;
    }

    position_error_previous = position_error;

    if (POSITION_KI > 0.0) {
        position_error_integral += position_error * time_difference;
        position_error_integral = fmax(fmin(position_error_integral, 1.0/POSITION_KI), -1.0/POSITION_KI);
    } else {
        position_error_integral = 0.0;
    }

    float pP = POSITION_KP * position_error * sensitivity_pP_gain;
    float pI = POSITION_KI * position_error_integral * sensitivity_pI_gain;
    float pD = POSITION_KD * position_error_diff * sensitivity_pD_gain;

    Q_position = pP + pI + pD;

    // Angle PID
    angle_error = angle;

    if (time_difference > 0.0001) {
        angle_error_diff = (angle_error - angle_error_previous) / time_difference;
    } else {
        angle_error_diff = 0.0;
    }

    angle_error_previous = angle_error;

    if (ANGLE_KI > 0.0) {
        angle_error_integral += angle_error * time_difference;
        angle_error_integral = fmax(fmin(angle_error_integral, 1.0/ANGLE_KI), -1.0/ANGLE_KI);
    } else {
        angle_error_integral = 0.0;
    }

    float aP = ANGLE_KP * angle_error * sensitivity_aP_gain;
    float aI = ANGLE_KI * angle_error_integral * sensitivity_aI_gain;
    float aD = ANGLE_KD * angle_error_diff * sensitivity_aD_gain;

    Q_angle = -aP - aI - aD;

    Q = Q_angle + Q_position;

    return Q;
}


void cmd_SetPIDConfig(const unsigned char * config)
{
	disable_irq();

	POSITION_KP         = *((float          *)&config[ 0]);
	POSITION_KI         = *((float          *)&config[ 4]);
	POSITION_KD         = *((float          *)&config[8]);

	ANGLE_KP            = *((float          *)&config[ 12]);
	ANGLE_KI            = *((float          *)&config[ 16]);
	ANGLE_KD            = *((float          *)&config[20]);

	enable_irq();
}


void cmd_GetPIDConfig(unsigned char * txBuffer)
{
	prepare_message_to_PC_config_PID(txBuffer, POSITION_KP, POSITION_KI, POSITION_KD, ANGLE_KP, ANGLE_KI, ANGLE_KD);

	disable_irq();
	Message_SendToPC(txBuffer, 34);
	enable_irq();
}

