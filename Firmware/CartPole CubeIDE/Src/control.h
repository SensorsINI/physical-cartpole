#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "sys.h"
#include "motor.h"

#define CONTROL_LOOP_PERIOD_MS	        5
#define CONTROL_SLOWDOWN				1
#define CONTROL_MOTOR_MAX_SPEED         ((int)(0.95 * MOTOR_FULL_SCALE + 0.5))
#define CONTROL_SYNC					false
#define CONTROL_LATENCY_US				0


/***** Angle Set Point *****/
#define CONTROL_ANGLE_SET_POINT_POLULU	    3355
#define CONTROL_ANGLE_SET_POINT_ORIGINAL	3163

/***** Angle & Position *****/
#define CONTROL_ANGLE_SMOOTHING		1.0f        	// 0 to 1.0 (1.0 disables smoothing)
#define CONTROL_ANGLE_AVERAGE_LEN	32  			// Number of samples to average over to determine angular displacement (max is 32)
#define CONTROL_ANGLE_MEASUREMENT_INTERVAL_US 200

#define CONTROL_POSITION_SET_POINT      0
#define CONTROL_POSITION_SMOOTHING      0.2f 		// 0.2f        // 0 to 1.0 (1.0 disables smoothing)
#define CONTROL_POSITION_PERIOD_MS      25 			// 25

/***** PID *****/
#define CONTROL_ANGLE_KP                200.0f		// 200.0f
#define CONTROL_ANGLE_KI                0			// 0.0f
#define CONTROL_ANGLE_KD                200.0f		// 200.0f

#define CONTROL_POSITION_KP             4.0f 		// 4.0f
#define CONTROL_POSITION_KI             0.0f		// 0.0f
#define CONTROL_POSITION_KD             150.0f 		// 150.0f


void CONTROL_Init(void);
void CONTROL_Calibrate(void);
void CONTROL_ToggleState(void);
void CONTROL_Loop(void);
void CONTROL_BackgroundTask(void);

#endif /*__CONTROL_H_*/
