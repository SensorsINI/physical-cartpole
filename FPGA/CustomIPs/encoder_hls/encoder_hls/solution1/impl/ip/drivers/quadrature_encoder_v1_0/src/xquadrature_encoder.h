// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XQUADRATURE_ENCODER_H
#define XQUADRATURE_ENCODER_H

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
#include "xquadrature_encoder_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#else
typedef struct {
    u16 DeviceId;
    u32 Encoder_axi_BaseAddress;
} XQuadrature_encoder_Config;
#endif

typedef struct {
    u32 Encoder_axi_BaseAddress;
    u32 IsReady;
} XQuadrature_encoder;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XQuadrature_encoder_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XQuadrature_encoder_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XQuadrature_encoder_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XQuadrature_encoder_ReadReg(BaseAddress, RegOffset) \
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
int XQuadrature_encoder_Initialize(XQuadrature_encoder *InstancePtr, u16 DeviceId);
XQuadrature_encoder_Config* XQuadrature_encoder_LookupConfig(u16 DeviceId);
int XQuadrature_encoder_CfgInitialize(XQuadrature_encoder *InstancePtr, XQuadrature_encoder_Config *ConfigPtr);
#else
int XQuadrature_encoder_Initialize(XQuadrature_encoder *InstancePtr, const char* InstanceName);
int XQuadrature_encoder_Release(XQuadrature_encoder *InstancePtr);
#endif


void XQuadrature_encoder_Set_reset(XQuadrature_encoder *InstancePtr, u32 Data);
u32 XQuadrature_encoder_Get_reset(XQuadrature_encoder *InstancePtr);
u32 XQuadrature_encoder_Get_count(XQuadrature_encoder *InstancePtr);
u32 XQuadrature_encoder_Get_count_vld(XQuadrature_encoder *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
