#include "motor_zynq.h"

#include "xil_io.h"
#include <stdlib.h>
#include <unistd.h>

XMotor_hls Motor_Instance;


void set_direction(int pwm_duty_cycle_in_clock_cycles);
void set_power_magnitude(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles);


void Motor_Init_Zynq(int pwm_period_in_clock_cycles)
{
	XMotor_hls_Initialize(&Motor_Instance, MOTOR_DEVICE_ID);
	Motor_SetPwmPeriod_Zynq(pwm_period_in_clock_cycles);
	Motor_Stop_Zynq();
	XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(&Motor_Instance, (u32)(0));

}

void Motor_SetPwmPeriod_Zynq(int pwm_period_in_clock_cycles){

	XMotor_hls_Set_pwm_period_in_clock_cycles(&Motor_Instance, (u32)(pwm_period_in_clock_cycles));
}

void Motor_Stop_Zynq(void)
{
	XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(&Motor_Instance, (u32)(0));
}


void Motor_SetPower_Zynq(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles)
{
	#ifdef POLOLU_MOTOR
	pwm_duty_cycle_in_clock_cycles=-pwm_duty_cycle_in_clock_cycles; // tobi
	#endif

	XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(&Motor_Instance, (u32)(pwm_duty_cycle_in_clock_cycles));
}


