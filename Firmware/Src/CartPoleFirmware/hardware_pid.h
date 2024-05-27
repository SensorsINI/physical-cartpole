/*
 * hardware_pid.h
 *
 *  Created on: 19 Dec 2023
 *      Author: marcinpaluch
 */

#ifndef HARDWARE_PID_H_
#define HARDWARE_PID_H_


float pid_step(float angle, float angleD, float position, float positionD, float target_position, float time);

#endif /* HARDWARE_PID_H_ */

void cmd_SetPIDConfig(const unsigned char * config);
void cmd_GetPIDConfig(unsigned char * txBuffer);
