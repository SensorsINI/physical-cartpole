#ifndef __CONTROL_H_
#define __CONTROL_H_


#define CONTROL_LOOP_PERIOD_MS	        5
#define CONTROL_SLOWDOWN				0
#define CONTROL_SYNC					false
#define CONTROL_LATENCY_US				0


/***** Angle Set Point *****/
#define CONTROL_ANGLE_SET_POINT_POLULU	    -3065.17
#define CONTROL_ANGLE_SET_POINT_ORIGINAL	-3065.17

/***** Angle & Position *****/
#define CONTROL_ANGLE_MEASUREMENT_INTERVAL_US 200

void CONTROL_Init(void);
void CONTROL_Calibrate(void);
void CONTROL_ToggleState(void);
void CONTROL_Loop(void);
void CONTROL_BackgroundTask(void);

#endif /*__CONTROL_H_*/
