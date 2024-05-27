#include <iostream>
#include "motor_hls.h"

int main() {
    int pwm_period = 100; // Example PWM period
    volatile bool pwm_out;
    volatile bool direction_output_1;
    volatile bool direction_output_2;

    // Test with a 50% duty cycle
    int pwm_duty = 50;
    std::cout << "Testing with 50% duty cycle." << std::endl;
    for (int i = 0; i < 2 * pwm_period; ++i) { // Run for two periods
        motor_hls(pwm_period, pwm_duty, &pwm_out, &direction_output_1, &direction_output_2);
        std::cout << "Cycle " << i << ": PWM Out = " << pwm_out
                  << ", Direction 1 = " << direction_output_1
                  << ", Direction 2 = " << direction_output_2 << std::endl;
    }

    // Test with a 75% duty cycle
    pwm_duty = 75;
    std::cout << "\nTesting with 75% duty cycle." << std::endl;
    for (int i = 0; i < 2 * pwm_period; ++i) { // Run for two periods
        motor_hls(pwm_period, pwm_duty, &pwm_out, &direction_output_1, &direction_output_2);
        std::cout << "Cycle " << i << ": PWM Out = " << pwm_out
                  << ", Direction 1 = " << direction_output_1
                  << ", Direction 2 = " << direction_output_2 << std::endl;
    }

    // Test with a negative duty cycle for reverse direction
    pwm_duty = -50;
    std::cout << "\nTesting with -50% duty cycle for reverse direction." << std::endl;
    for (int i = 0; i < 2 * pwm_period; ++i) { // Run for two periods
        motor_hls(pwm_period, pwm_duty, &pwm_out, &direction_output_1, &direction_output_2);
        std::cout << "Cycle " << i << ": PWM Out = " << pwm_out
                  << ", Direction 1 = " << direction_output_1
                  << ", Direction 2 = " << direction_output_2 << std::endl;
    }

    return 0;
}
