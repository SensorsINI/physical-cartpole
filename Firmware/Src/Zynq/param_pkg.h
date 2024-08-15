#ifndef __PARAM_PKG_H_
#define __PARAM_PKG_H_
/************ Macro Definitions ************/

#include "xparameters.h"

#define GPIO_MOTOR_BASEADDR     XPAR_PMODDHB1_0_AXI_LITE_GPIO_BASEADDR
#define PWM_BASEADDR      XPAR_PMODDHB1_0_PWM_AXI_BASEADDR
#define MOTOR_FB_BASEADDR XPAR_PMODDHB1_0_MOTOR_FB_AXI_BASEADDR

#ifdef __MICROBLAZE__
#define CLK_FREQ XPAR_CPU_M_AXI_DP_FREQ_HZ
#else
#define CLK_FREQ 25000000 // FCLK0 frequency not found in xparameters.h
#endif

#define SENSOR_EDGES_PER_REV 64 //4
#define GEARBOX_RATIO        18.75//48

#define SYSMON_DEVICE_ID  XPAR_SYSMON_0_DEVICE_ID
#define XSysMon_RawToExtVoltage(AdcData)  ((((float)(AdcData))*(1.0f))/65536.0f)

#endif /*__PARAM_PKG_H_*/
