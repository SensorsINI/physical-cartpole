#include "hardware_bridge.h"  // From here we only need to know if STM or Zynq firmware was selected.
#include "parameters.h"
#include <stdbool.h>


const unsigned int UART_BAUD	=	230400; 	// 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000 // Not working for Zynq yet


unsigned short CONTROL_LOOP_PERIOD_MS				=		1;
unsigned short CONTROL_SLOWDOWN						=		0;
bool CONTROL_SYNC									=		true;
unsigned short ANGLE_MEASUREMENT_INTERVAL_US		= 		200;

const unsigned short ANGLE_AVERAGE_LEN_MAX			=		32;
unsigned short ANGLE_AVERAGE_LEN					=		1;		// Number of samples to average over to determine angular displacement (max is 32)
unsigned short MAX_ADC_STEP							=		20;		// If jump between two consecutive ADC measurements is bigger than this, the measurement counts as invalid


#ifdef STM

float MOTOR_CORRECTION[3] 							=		{4898.18, 168.09, 123.46}; // Pololu

float CONTROL_ANGLE_SET_POINT_POLULU				=		-3065.17;
float CONTROL_ANGLE_SET_POINT_ORIGINAL				=		-3065.17;

float ANGLE_NORMALIZATION_FACTOR					=		0.001471127442561364;	// Check this
float POSITION_NORMALIZATION_FACTOR 				=		0.00009510086455331;	// Check this

const unsigned int CLOCK_FREQ						=		72000000;
const int PWM_PERIOD_IN_CLOCK_CYCLES		=		7200;

const float MOTOR_FULL_SCALE					=		(PWM_PERIOD_IN_CLOCK_CYCLES-1);
const int MOTOR_FULL_SCALE_SAFE			=		((int)(0.95 * MOTOR_FULL_SCALE + 0.5));

bool USE_TARGET_SWITCHES							=		false;					// Needs to be always false for STM


#elif defined(ZYNQ)

float MOTOR_CORRECTION[3] 							=		{1488.070,  80.797,  96.254}; // Pololu

float CONTROL_ANGLE_SET_POINT_POLULU				=		-3042.835;
float CONTROL_ANGLE_SET_POINT_ORIGINAL				=		-3042.835;

float ANGLE_NORMALIZATION_FACTOR					=		0.0015442848171956895;
float POSITION_NORMALIZATION_FACTOR					=		0.0000841657806278922;

const unsigned int CLOCK_FREQ						=		333333343;
const int PWM_PERIOD_IN_CLOCK_CYCLES		=		2500;

const float MOTOR_FULL_SCALE					=		(PWM_PERIOD_IN_CLOCK_CYCLES-1);
const int MOTOR_FULL_SCALE_SAFE			=		((int)(0.95 * MOTOR_FULL_SCALE + 0.5));

bool USE_TARGET_SWITCHES							=		true;					// You can set it true or false

#endif
