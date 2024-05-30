#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// See parameters.c to set values. Here only declarations.

#include <stdbool.h>

extern const unsigned int UART_BAUD;

extern unsigned short CONTROL_LOOP_PERIOD_MS;
extern unsigned short CONTROL_SLOWDOWN;
extern bool CONTROL_SYNC;
extern unsigned short ANGLE_MEASUREMENT_INTERVAL_US;

extern const unsigned short ANGLE_AVERAGE_LEN_MAX;
extern unsigned short ANGLE_AVERAGE_LEN;				// Number of samples to average over to determine angular displacement (max is 32)
extern unsigned short MAX_ADC_STEP;						// If jump between two consecutive ADC measurements is bigger than this, the measurement counts as invalid

extern int MOTOR;

extern float MOTOR_CORRECTION[3];

typedef enum {
    MOTOR_ORIGINAL,
    MOTOR_POLOLU,
} MOTOR_enum;

extern float CONTROL_ANGLE_SET_POINT_POLULU;
extern float CONTROL_ANGLE_SET_POINT_ORIGINAL;

extern float ANGLE_NORMALIZATION_FACTOR;
extern float POSITION_NORMALIZATION_FACTOR;

extern const unsigned int CLOCK_FREQ;
extern const int PWM_PERIOD_IN_CLOCK_CYCLES;

extern const float MOTOR_FULL_SCALE;
extern const int MOTOR_FULL_SCALE_SAFE;

extern bool USE_TARGET_SWITCHES;

#endif /* PARAMETERS_H_ */
