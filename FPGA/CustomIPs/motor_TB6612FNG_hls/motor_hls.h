#ifndef MOTOR_HLS_H_
#define MOTOR_HLS_H_

// Define the interface for the motor_hls function which controls a motor using PWM.
void motor_hls(
    int pwm_period_in_clock_cycles,
    int pwm_duty_cycle_in_clock_cycles,
    volatile bool *pwm_out,
    volatile bool *direction_output_1,
    volatile bool *direction_output_2
);

#endif // MOTOR_HLS_H_
