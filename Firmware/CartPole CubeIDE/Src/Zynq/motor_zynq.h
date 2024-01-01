#ifndef __MOTOR_ZYNQ_H_
#define __MOTOR_ZYNQ_H_

#include "xmotor_hls.h"
#include "param_pkg.h"

#define MOTOR_DEVICE_ID XPAR_MOTOR_HLS_0_DEVICE_ID

void Motor_Init_Zynq(int pwm_period_in_clock_cycles);
void Motor_SetPwmPeriod_Zynq(int pwm_period_in_clock_cycles);
void Motor_Stop_Zynq(void);
void Motor_SetPower_Zynq(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles);

#endif /*__MOTOR_ZYNQ_H_*/
