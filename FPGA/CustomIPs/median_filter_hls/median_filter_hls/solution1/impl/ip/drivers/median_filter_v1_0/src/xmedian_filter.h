// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XMEDIAN_FILTER_H
#define XMEDIAN_FILTER_H

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
#include "xmedian_filter_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Median_BaseAddress;
} XMedian_filter_Config;
#endif

typedef struct {
    u32 Median_BaseAddress;
    u32 IsReady;
} XMedian_filter;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XMedian_filter_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XMedian_filter_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XMedian_filter_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XMedian_filter_ReadReg(BaseAddress, RegOffset) \
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
int XMedian_filter_Initialize(XMedian_filter *InstancePtr, u16 DeviceId);
XMedian_filter_Config* XMedian_filter_LookupConfig(u16 DeviceId);
int XMedian_filter_CfgInitialize(XMedian_filter *InstancePtr, XMedian_filter_Config *ConfigPtr);
#else
int XMedian_filter_Initialize(XMedian_filter *InstancePtr, const char* InstanceName);
int XMedian_filter_Release(XMedian_filter *InstancePtr);
#endif


void XMedian_filter_Set_median_i(XMedian_filter *InstancePtr, u32 Data);
u32 XMedian_filter_Get_median_i(XMedian_filter *InstancePtr);
u32 XMedian_filter_Get_median_o(XMedian_filter *InstancePtr);
u32 XMedian_filter_Get_median_o_vld(XMedian_filter *InstancePtr);
void XMedian_filter_Set_window_size(XMedian_filter *InstancePtr, u32 Data);
u32 XMedian_filter_Get_window_size(XMedian_filter *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
