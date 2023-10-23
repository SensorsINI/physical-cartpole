#ifndef __ENCODER_STM_H_
#define __ENCODER_STM_H_

#include "sys.h"

#define ENCODER_ZERO	0 // 10000U

void  Encoder_Init_STM(void);
short	Encoder_Read_STM(void);

#endif /*__ENCODER_STM_H_*/
