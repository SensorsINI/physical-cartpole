#include "hardware_bridge.h"  // From here we only need to know if STM or Zynq firmware was selected.
#include "parameters.h"
#include <stdbool.h>
#include "math.h"


const unsigned int UART_BAUD	=	230400; 	// 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000 // Not working for Zynq yet


unsigned short POLLING_PERIOD_MS				=		10;  /* CONTROL_Init replaces this with the boot controller default; the PC may overwrite via CMD_SET_CONTROL_CONFIG. */
unsigned short CONTROL_SLOWDOWN						=		0;
bool CONTROL_SYNC									=		true;

// Calculating derivatives and dead angle detection
unsigned short TIMESTEPS_FOR_DERIVATIVE				=		10;  // IROS short pole chip: 10 × 1 ms = 10 ms. PC k uses N=2 at 5 ms.
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
#if IROS_SHORT_POLE_PROFILE
/* Old IROS firmware used 1.25 * 0.09 m. */
const float SliderTargetHalfLength					=		0.1125;
#else
const float SliderTargetHalfLength					=		0.14;  // JB pot rails; inside the 0.198 m track
#endif

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

#ifdef ZEDBOARD
// Calibration for the Zedboard-lab physical cartpole. Not the Development default.
float MOTOR_CORRECTION[3] 							=		{0.5846884, 0.0223145, 0.0224489};

float ANGLE_HANGING_POLOLU 							=		1068;
float ANGLE_HANGING_ORIGINAL						=		1075;

const float ANGLE_360_DEG_IN_ADC_UNITS				=		4302;
const float POSITION_ENCODER_RANGE					=		4649;

#else  // ZYBO_Z720

#ifdef RPGD_DUAL_CORE
/* 2026-09-02 go-to: same LSTM quant map as working PC rpgd-c at 20 ms. */
float MOTOR_CORRECTION[3] 							=		{0.5733488, 0.0257380, 0.0258429};
#elif IROS_SHORT_POLE_PROFILE
/*
 * PL-only short-pole map selected by physical comparison on Development.
 * The PC path keeps its independently configured map in Driver/globals.py.
 */
float MOTOR_CORRECTION[3] 							=		{0.5733488, 0.0257380, 0.0258429};
#else
/* Dense-8 go-to da41c737: 0.573 (force-fit 0.511 too weak). */
float MOTOR_CORRECTION[3] 							=		{0.5733488, 0.0257380, 0.0258429};
#endif

float ANGLE_HANGING_POLOLU 							=		3273.353;  // Stationary hanging mean, new analog chain 2026-09-03
float ANGLE_HANGING_ORIGINAL						=		1008.5;  // Value from sensor when pendulum is at stable equilibrium point

const float ANGLE_360_DEG_IN_ADC_UNITS				=		4068.73;  // 2*(upright 1238.988 - hanging 3273.353); must match Driver/globals.py.
#if IROS_SHORT_POLE_PROFILE
const float POSITION_ENCODER_RANGE					=		4705.0;
#else
const float POSITION_ENCODER_RANGE					=		4695.0;
#endif

#endif

const unsigned int CLOCK_FREQ						=		333333343;
#if IROS_SHORT_POLE_PROFILE && !defined(RPGD_DUAL_CORE)
const int MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES			=		2500;
#else
const int MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES			=		10000;
#endif

const int MOTOR_FULL_SCALE_SAFE						=		((int)(0.95 * MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES + 0.5));

bool USE_TARGET_SWITCHES							=		false;					// Keep switch positions from changing target during LQR tests

#endif

float ANGLE_NORMALIZATION_FACTOR					=		((2 * M_PI) / ANGLE_360_DEG_IN_ADC_UNITS);
float POSITION_NORMALIZATION_FACTOR					=		(TrackHalfLength * 2 / POSITION_ENCODER_RANGE);

const float DEAD_ZONE_VERTICAL_WARN_DEG				=		20.0f;
