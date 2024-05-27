#include <hls_math.h>
#include "motor_hls.h"


void motor_hls(
    int pwm_period_in_clock_cycles,
    int pwm_duty_cycle_in_clock_cycles,
    volatile bool *pwm_out,
    volatile bool *direction_output_1,
	volatile bool *direction_output_2
) {

	// Define the I/O ports properties for Vivado HLS
	#pragma HLS INTERFACE ap_ctrl_none port=return
	#pragma HLS INTERFACE s_axilite port=pwm_period_in_clock_cycles bundle=MOTOR_AXI
	#pragma HLS INTERFACE s_axilite port=pwm_duty_cycle_in_clock_cycles bundle=MOTOR_AXI
	#pragma HLS INTERFACE ap_none port=pwm_out
	#pragma HLS INTERFACE ap_none port=direction_output_1
	#pragma HLS INTERFACE ap_none port=direction_output_2

	static int counter = 0;

	*direction_output_1 = pwm_duty_cycle_in_clock_cycles > 0;  // AIN1
	*direction_output_2 = pwm_duty_cycle_in_clock_cycles < 0;   // AIN2

	pwm_period_in_clock_cycles = (pwm_period_in_clock_cycles <= 0) ? 1 : pwm_period_in_clock_cycles;
    // Ensure duty cycle is positive for comparison
    int duty_cycle = hls::abs(pwm_duty_cycle_in_clock_cycles);

	// Generate PWM signal based on counter and duty cycle
    duty_cycle = (duty_cycle > pwm_period_in_clock_cycles) ? pwm_period_in_clock_cycles : duty_cycle;

	*pwm_out = (counter < duty_cycle);

    // Increment the counter and reset it if it exceeds the period
    counter = (counter >= pwm_period_in_clock_cycles-1) ? 0 : counter + 1;
}



