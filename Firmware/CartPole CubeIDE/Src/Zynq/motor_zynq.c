#include "motor_zynq.h"

#include "xil_io.h"
#include <stdlib.h>
#include <unistd.h>

XMotor_hls Motor_Instance;


void set_direction(int pwm_duty_cycle_in_clock_cycles);
void set_power_magnitude(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles);


void Motor_INIT(int pwm_period_in_clock_cycles)
{
	XMotor_hls_Initialize(&Motor_Instance, MOTOR_DEVICE_ID);
	Motor_SetPwmPeriod(pwm_period_in_clock_cycles);
	Motor_Stop();
	XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(&Motor_Instance, (u32)(0));

}

void Motor_SetPwmPeriod(int pwm_period_in_clock_cycles){

	XMotor_hls_Set_pwm_period_in_clock_cycles(&Motor_Instance, (u32)(pwm_period_in_clock_cycles));
}

void Motor_Stop(void)
{
	XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(&Motor_Instance, (u32)(0));
}


void Motor_SetPower(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles)
{
	XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(&Motor_Instance, (u32)(pwm_duty_cycle_in_clock_cycles));
}


