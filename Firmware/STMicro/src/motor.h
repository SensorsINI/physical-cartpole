#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "sys.h" 

#define MOTOR_FULL_SCALE    7199

void MOTOR_Init(void);
void MOTOR_Stop(void);
void MOTOR_SetSpeed(int speed);

#endif /*__MOTOR_H_*/
