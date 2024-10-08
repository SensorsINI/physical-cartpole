#include "hardware_bridge.h"  // From here we only need to know if STM or Zynq firmware was selected.
#include "parameters.h"
#include <stdbool.h>
#include "math.h"


const unsigned int UART_BAUD	=	230400; 	// 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000 // Not working for Zynq yet


unsigned short CONTROL_LOOP_PERIOD_MS				=		20;
unsigned short CONTROL_SLOWDOWN						=		0;
bool CONTROL_SYNC									=		true;

// Calculating derivatives and dead angle detection
unsigned short TIMESTEPS_FOR_DERIVATIVE				=		1;  // 20 at most
// TIMESTEPS_FOR_DERIVATIVE: How many timesteps are taken for derivative (position and angle) calculation
// and dead angle detection.
// Too small value makes the effect of sensor quantization severe.
// Too big causes derivatives to represent old and strongly averaged information
unsigned short MAX_ADC_STEP							=		20;		// If jump between two consecutive ADC measurements is bigger than this, the measurement counts as invalid

// Firmware filter - in general not active on Zynq, it has a hardware filter
unsigned short ANGLE_MEASUREMENT_INTERVAL_US		= 		200;
const unsigned short ANGLE_AVERAGE_LEN_MAX			=		32;
unsigned short ANGLE_AVERAGE_LEN					=		1;		// Number of samples to average over to determine angular displacement (max is 32)


const float TrackHalfLength							=		0.198;

int MOTOR = MOTOR_POLOLU;

#ifdef STM

float MOTOR_CORRECTION[3] 							=		{0.595228, 0.0323188, 0.0385016}; // Pololu

float ANGLE_HANGING_POLOLU 							=		1055.5;  // Value from sensor when pendulum is at stable equilibrium point
float ANGLE_HANGING_ORIGINAL						=		1046.75;  // Value from sensor when pendulum is at stable equilibrium point

const float ANGLE_360_DEG_IN_ADC_UNITS				=		4271.34;
const float POSITION_ENCODER_RANGE					=		4164.0;

const unsigned int CLOCK_FREQ						=		72000000;
const int MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES				=		7200;

const int MOTOR_FULL_SCALE_SAFE						=		((int)(0.95 * MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES + 0.5));

bool USE_TARGET_SWITCHES							=		false;					// Needs to be always false for STM


#elif defined(ZYNQ)

float MOTOR_CORRECTION[3] 							=		{0.5733488, 0.0257380, 0.0258429}; // Pololu

float ANGLE_HANGING_POLOLU 							=		1250.0;  // Value from sensor when pendulum is at stable equilibrium point
float ANGLE_HANGING_ORIGINAL						=		1008.5;  // Value from sensor when pendulum is at stable equilibrium point

const float ANGLE_360_DEG_IN_ADC_UNITS				=		4069.05;
const float POSITION_ENCODER_RANGE					=		4695.0;

const unsigned int CLOCK_FREQ						=		333333343;
const int MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES			=		10000;

const int MOTOR_FULL_SCALE_SAFE						=		((int)(0.95 * MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES + 0.5));

bool USE_TARGET_SWITCHES							=		true;					// You can set it true or false

#endif

float ANGLE_NORMALIZATION_FACTOR					=		((2 * M_PI) / ANGLE_360_DEG_IN_ADC_UNITS);
float POSITION_NORMALIZATION_FACTOR					=		(TrackHalfLength * 2 / POSITION_ENCODER_RANGE);
