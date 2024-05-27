// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XMOTOR_HLS_H
#define XMOTOR_HLS_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xmotor_hls_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#else
typedef struct {
    u16 DeviceId;
    u32 Motor_axi_BaseAddress;
} XMotor_hls_Config;
#endif

typedef struct {
    u32 Motor_axi_BaseAddress;
    u32 IsReady;
} XMotor_hls;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XMotor_hls_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XMotor_hls_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XMotor_hls_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XMotor_hls_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XMotor_hls_Initialize(XMotor_hls *InstancePtr, u16 DeviceId);
XMotor_hls_Config* XMotor_hls_LookupConfig(u16 DeviceId);
int XMotor_hls_CfgInitialize(XMotor_hls *InstancePtr, XMotor_hls_Config *ConfigPtr);
#else
int XMotor_hls_Initialize(XMotor_hls *InstancePtr, const char* InstanceName);
int XMotor_hls_Release(XMotor_hls *InstancePtr);
#endif


void XMotor_hls_Set_pwm_period_in_clock_cycles(XMotor_hls *InstancePtr, u32 Data);
u32 XMotor_hls_Get_pwm_period_in_clock_cycles(XMotor_hls *InstancePtr);
void XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(XMotor_hls *InstancePtr, u32 Data);
u32 XMotor_hls_Get_pwm_duty_cycle_in_clock_cycles(XMotor_hls *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
