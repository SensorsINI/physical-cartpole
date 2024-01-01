#ifndef __ANGLE_ZYNQ_H_
#define __ANGLE_ZYNQ_H_

#include "xsysmon.h"
#include "xadcps.h"
#include <stdio.h>
#include "param_pkg.h"

#define XADC_DEVICE_ID XPAR_PS7_XADC_0_DEVICE_ID


void             Goniometer_Init_Zynq(void);
unsigned short 	Goniometer_Read_Zynq(void);

#endif /*__ANGLE_ZYNQ_H_*/
