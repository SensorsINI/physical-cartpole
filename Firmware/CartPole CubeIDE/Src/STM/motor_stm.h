#ifndef __MOTOR_STM_H_
#define __MOTOR_STM_H_

#include "sys.h" 

void Motor_INIT(int pwm_period_in_clock_cycles);
void Motor_SetPwmPeriod(int pwm_period_in_clock_cycles);
void Motor_Stop(void);
void Motor_SetPower(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles);

#endif /*__MOTOR_STM_H_*/
