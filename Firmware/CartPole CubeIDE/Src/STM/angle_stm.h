#ifndef __ANGLE_STM_H_
#define __ANGLE_STM_H_

#include "sys.h"

void            Goniometer_Init_STM(void);
unsigned short 	Goniometer_Read_STM(void);
unsigned short 	Goniometer_ReadAvergage_STM(unsigned int n);

#endif /*__ANGLE_STM_H_*/
