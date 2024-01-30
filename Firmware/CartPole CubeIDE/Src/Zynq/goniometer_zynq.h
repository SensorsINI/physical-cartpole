#ifndef __GONIOMETER_ZYNQ_H_
#define __GONIOMETER_ZYNQ_H_

#include "xsysmon.h"
#include "xadcps.h"
#include <stdio.h>
#include "param_pkg.h"

#define XADC_DEVICE_ID XPAR_PS7_XADC_0_DEVICE_ID

#include "xmedian_filter.h"
#define MEDIAN_FILTER_DEVICE_ID XPAR_MEDIAN_FILTER_0_DEVICE_ID

// Hardware Median can be 1(no averaging) to 64 (but don't take even numbers)
// It always causes xxx us delay (as long as max average is not changed in hardware)
#define HARDWARE_MEDIAN 63


void             Goniometer_Init(void);
unsigned short 	Goniometer_Read(void);

#endif /*__GONIOMETER_ZYNQ_H_*/
