#ifndef __GONIOMETER_ZYNQ_H_
#define __GONIOMETER_ZYNQ_H_

#include "xsysmon.h"
#include "xadcps.h"
#include <stdio.h>
#include "param_pkg.h"

#define XADC_DEVICE_ID XPAR_PS7_XADC_0_DEVICE_ID


void             Goniometer_Init(void);
unsigned short 	Goniometer_Read(void);

#endif /*__GONIOMETER_ZYNQ_H_*/
