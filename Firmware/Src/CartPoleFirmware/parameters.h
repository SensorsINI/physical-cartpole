#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// See parameters.c to set values. Here only declarations.

#include <stdbool.h>

/*
 * Legacy compile-time pole profile. Zybo show mux applies timing/maps at
 * runtime (controller_profiles.c); leave this 0 so leftover #if branches
 * match the long-pole idle default.
 */
#define IROS_SHORT_POLE_PROFILE 0

extern const unsigned int UART_BAUD;

extern unsigned short POLLING_PERIOD_MS;
extern unsigned short CONTROL_SLOWDOWN;
extern bool CONTROL_SYNC;
extern unsigned short ANGLE_MEASUREMENT_INTERVAL_US;

extern unsigned short TIMESTEPS_FOR_DERIVATIVE;
extern unsigned short MAX_ADC_STEP;						// If jump between two consecutive ADC measurements is bigger than this, the measurement counts as invalid


extern const unsigned short ANGLE_AVERAGE_LEN_MAX;
extern unsigned short ANGLE_AVERAGE_LEN;				// Number of samples to average over to determine angular displacement (max is 32)

extern int MOTOR;

extern float MOTOR_CORRECTION[3];

typedef enum {
    MOTOR_ORIGINAL,
    MOTOR_POLOLU,
} MOTOR_enum;


extern const float TrackHalfLength;
extern float SliderTargetHalfLength;
extern float ANGLE_360_DEG_IN_ADC_UNITS;
extern float POSITION_ENCODER_RANGE;

extern float ANGLE_HANGING_POLOLU;
extern float ANGLE_HANGING_ORIGINAL;

extern float ANGLE_NORMALIZATION_FACTOR;
extern float POSITION_NORMALIZATION_FACTOR;

extern const unsigned int CLOCK_FREQ;
extern int MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES;

extern int MOTOR_FULL_SCALE_SAFE;

extern bool USE_TARGET_SWITCHES;

/* Warn when the pot dead zone (ADC wrap) is this close to vertical up or down. */
extern const float DEAD_ZONE_VERTICAL_WARN_DEG;

#endif /* PARAMETERS_H_ */
