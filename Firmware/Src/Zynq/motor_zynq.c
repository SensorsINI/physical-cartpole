#include "motor_zynq.h"

#include "hardware_bridge.h"  // Board selection (ZEDBOARD/ZYBO_Z720); needed for the polarity fix below
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
#ifdef ZEDBOARD
	// The Zedboard rig in the Sevilla lab has the motor wired with reversed
	// polarity: a positive duty cycle drove the cart to the left, while the
	// controller/simulator convention (and the encoder, which counts up to the
	// right) require positive = right. Invert the command to compensate.
	// Verified 2026-07-03 by pulsing the motor and observing cart direction and
	// raw encoder counts; with this inversion calibration detects POLOLU again.
	pwm_duty_cycle_in_clock_cycles = -pwm_duty_cycle_in_clock_cycles;
#endif
	XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(&Motor_Instance, (u32)(pwm_duty_cycle_in_clock_cycles));
}


