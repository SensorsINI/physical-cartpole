#ifndef __MOTOR_STM_H_
#define __MOTOR_STM_H_

#include "sys.h" 

void Motor_Init_STM(int pwm_period_in_clock_cycles);
void Motor_SetPwmPeriod_STM(int pwm_period_in_clock_cycles);
void Motor_Stop_STM(void);
void Motor_SetPower_STM(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles);

#endif /*__MOTOR_STM_H_*/
