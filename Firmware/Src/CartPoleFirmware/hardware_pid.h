/*
 * hardware_pid.h
 *
 *  Created on: 19 Dec 2023
 *      Author: marcinpaluch
 */

#ifndef HARDWARE_PID_H_
#define HARDWARE_PID_H_


float pid_step(float angle, float angleD, float position, float positionD, float target_position, float time);

typedef struct {
    float error_previous;
    float error_integral;
} PIDState;

float pid_core(PIDState *pid_state, float error, float time_difference,
		float KP, float KI, float KD,
		float sensitivity_P_gain, float sensitivity_I_gain, float sensitivity_D_gain);

void cmd_SetPIDConfig(const unsigned char * config);
void cmd_GetPIDConfig(unsigned char * txBuffer);

#endif /* HARDWARE_PID_H_ */
