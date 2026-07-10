// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XSECLOC_SHELL_H
#define XSECLOC_SHELL_H

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
#include "xsecloc_shell_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Ctrl_BaseAddress;
} XSecloc_shell_Config;
#endif

typedef struct {
    u32 Ctrl_BaseAddress;
    u32 IsReady;
} XSecloc_shell;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XSecloc_shell_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XSecloc_shell_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XSecloc_shell_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XSecloc_shell_ReadReg(BaseAddress, RegOffset) \
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
int XSecloc_shell_Initialize(XSecloc_shell *InstancePtr, u16 DeviceId);
XSecloc_shell_Config* XSecloc_shell_LookupConfig(u16 DeviceId);
int XSecloc_shell_CfgInitialize(XSecloc_shell *InstancePtr, XSecloc_shell_Config *ConfigPtr);
#else
int XSecloc_shell_Initialize(XSecloc_shell *InstancePtr, const char* InstanceName);
int XSecloc_shell_Release(XSecloc_shell *InstancePtr);
#endif

void XSecloc_shell_Start(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_IsDone(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_IsIdle(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_IsReady(XSecloc_shell *InstancePtr);
void XSecloc_shell_EnableAutoRestart(XSecloc_shell *InstancePtr);
void XSecloc_shell_DisableAutoRestart(XSecloc_shell *InstancePtr);

void XSecloc_shell_Set_angleD(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_angleD(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_angle_cos(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_angle_cos(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_angle_sin(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_angle_sin(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_position(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_position(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_positionD(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_positionD(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_target_equilibrium(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_target_equilibrium(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_target_position(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_target_position(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_angle(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_angle(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_tick(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_tick(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_log_base(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_log_base(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_ref_period_ticks(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_ref_period_ticks(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_dead_ang(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_dead_ang(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_dead_pos(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_dead_pos(XSecloc_shell *InstancePtr);
void XSecloc_shell_Set_control_flags(XSecloc_shell *InstancePtr, u32 Data);
u32 XSecloc_shell_Get_control_flags(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_Get_Q(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_Get_Q_vld(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_Get_status(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_Get_status_vld(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_Get_update_count(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_Get_update_count_vld(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_Get_nn_wait_cycles(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_Get_nn_wait_cycles_vld(XSecloc_shell *InstancePtr);

void XSecloc_shell_InterruptGlobalEnable(XSecloc_shell *InstancePtr);
void XSecloc_shell_InterruptGlobalDisable(XSecloc_shell *InstancePtr);
void XSecloc_shell_InterruptEnable(XSecloc_shell *InstancePtr, u32 Mask);
void XSecloc_shell_InterruptDisable(XSecloc_shell *InstancePtr, u32 Mask);
void XSecloc_shell_InterruptClear(XSecloc_shell *InstancePtr, u32 Mask);
u32 XSecloc_shell_InterruptGetEnabled(XSecloc_shell *InstancePtr);
u32 XSecloc_shell_InterruptGetStatus(XSecloc_shell *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
