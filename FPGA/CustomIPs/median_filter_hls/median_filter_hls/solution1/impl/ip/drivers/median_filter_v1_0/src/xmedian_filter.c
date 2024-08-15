// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xmedian_filter.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XMedian_filter_CfgInitialize(XMedian_filter *InstancePtr, XMedian_filter_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Median_BaseAddress = ConfigPtr->Median_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XMedian_filter_Set_median_i(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_MEDIAN_I_DATA, Data);
}

u32 XMedian_filter_Get_median_i(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_MEDIAN_I_DATA);
    return Data;
}

u32 XMedian_filter_Get_median_o(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_MEDIAN_O_DATA);
    return Data;
}

u32 XMedian_filter_Get_median_o_vld(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_MEDIAN_O_CTRL);
    return Data & 0x1;
}

void XMedian_filter_Set_window_size(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_WINDOW_SIZE_DATA, Data);
}

u32 XMedian_filter_Get_window_size(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_WINDOW_SIZE_DATA);
    return Data;
}

