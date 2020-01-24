#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "sys.h"

#define ENCODER_ZERO	10000U

void            ENCODER_Init(void);
short	ENCODER_Read(void);

#endif /*__ENCODER_H_*/
