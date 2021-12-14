#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "sys.h"
#include "motor.h"

#define CONTROL_LOOP_PERIOD_MS	        5
#define CONTROL_MOTOR_MAX_SPEED         ((int)(0.95 * MOTOR_FULL_SCALE + 0.5))
#define CONTROL_SYNC					true
#define CONTROL_LATENCY_US				0

// Defaults
// set it to exactly the vertical position of the pendulum, by printing values and then averaging them

#define CONTROL_ANGLE_SET_POINT	    3110	// factory default
#define CONTROL_ANGLE_SET_POINT_LEFT	    3348	// left cartpole
#define CONTROL_ANGLE_SET_POINT_RIGHT	    3150	// right cartpole

#define CONTROL_ANGLE_SMOOTHING         1.0f        // 0 to 1.0 (1.0 disables smoothing)
#define CONTROL_ANGLE_KP                200.0f
#define CONTROL_ANGLE_KI                0
#define CONTROL_ANGLE_KD                200.0f
#define CONTROL_ANGLE_AVERAGE_LEN		32  //16        // Number of samples to average over to determine angular displacement (max is 32)
#define CONTROL_ANGLE_MEASUREMENT_INTERVAL_US	100

#define CONTROL_POSITION_SET_POINT      0
#define CONTROL_POSITION_SMOOTHING      0.2f // 0.2f        // 0 to 1.0 (1.0 disables smoothing)
#define CONTROL_POSITION_KP             4.0 //20.0f
#define CONTROL_POSITION_KI             0
#define CONTROL_POSITION_KD             150.0 //300.0f
#define CONTROL_POSITION_PERIOD_MS      25 // 25


void CONTROL_Init(void);
void CONTROL_Calibrate(void);
void CONTROL_ToggleState(void);
void CONTROL_Loop(void);
void CONTROL_BackgroundTask(void);

#endif /*__CONTROL_H_*/
