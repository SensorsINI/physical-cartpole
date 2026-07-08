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

void XMedian_filter_Set_filtered_i(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_FILTERED_I_DATA, Data);
}

u32 XMedian_filter_Get_filtered_i(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_FILTERED_I_DATA);
    return Data;
}

u32 XMedian_filter_Get_filtered_o(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_FILTERED_O_DATA);
    return Data;
}

u32 XMedian_filter_Get_filtered_o_vld(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_FILTERED_O_CTRL);
    return Data & 0x1;
}

void XMedian_filter_Set_raw_i(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_RAW_I_DATA, Data);
}

u32 XMedian_filter_Get_raw_i(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_RAW_I_DATA);
    return Data;
}

u32 XMedian_filter_Get_raw_o(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_RAW_O_DATA);
    return Data;
}

u32 XMedian_filter_Get_raw_o_vld(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_RAW_O_CTRL);
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

void XMedian_filter_Set_trim_count(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_TRIM_COUNT_DATA, Data);
}

u32 XMedian_filter_Get_trim_count(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_TRIM_COUNT_DATA);
    return Data;
}

void XMedian_filter_Set_filter_mode(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_FILTER_MODE_DATA, Data);
}

u32 XMedian_filter_Get_filter_mode(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_FILTER_MODE_DATA);
    return Data;
}

void XMedian_filter_Set_rail_low(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_RAIL_LOW_DATA, Data);
}

u32 XMedian_filter_Get_rail_low(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_RAIL_LOW_DATA);
    return Data;
}

void XMedian_filter_Set_rail_high(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_RAIL_HIGH_DATA, Data);
}

u32 XMedian_filter_Get_rail_high(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_RAIL_HIGH_DATA);
    return Data;
}

void XMedian_filter_Set_dz_status_i(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_STATUS_I_DATA, Data);
}

u32 XMedian_filter_Get_dz_status_i(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_STATUS_I_DATA);
    return Data;
}

u32 XMedian_filter_Get_dz_status_o(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_STATUS_O_DATA);
    return Data;
}

u32 XMedian_filter_Get_dz_status_o_vld(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_STATUS_O_CTRL);
    return Data & 0x1;
}

void XMedian_filter_Set_dz_window_i(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_WINDOW_I_DATA, Data);
}

u32 XMedian_filter_Get_dz_window_i(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_WINDOW_I_DATA);
    return Data;
}

u32 XMedian_filter_Get_dz_window_o(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_WINDOW_O_DATA);
    return Data;
}

u32 XMedian_filter_Get_dz_window_o_vld(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_WINDOW_O_CTRL);
    return Data & 0x1;
}

void XMedian_filter_Set_dz_age_i(XMedian_filter *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMedian_filter_WriteReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_AGE_I_DATA, Data);
}

u32 XMedian_filter_Get_dz_age_i(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_AGE_I_DATA);
    return Data;
}

u32 XMedian_filter_Get_dz_age_o(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_AGE_O_DATA);
    return Data;
}

u32 XMedian_filter_Get_dz_age_o_vld(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_AGE_O_CTRL);
    return Data & 0x1;
}

u32 XMedian_filter_Get_dz_low_count(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_LOW_COUNT_DATA);
    return Data;
}

u32 XMedian_filter_Get_dz_low_count_vld(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_LOW_COUNT_CTRL);
    return Data & 0x1;
}

u32 XMedian_filter_Get_dz_high_count(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_HIGH_COUNT_DATA);
    return Data;
}

u32 XMedian_filter_Get_dz_high_count_vld(XMedian_filter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMedian_filter_ReadReg(InstancePtr->Median_BaseAddress, XMEDIAN_FILTER_MEDIAN_ADDR_DZ_HIGH_COUNT_CTRL);
    return Data & 0x1;
}

