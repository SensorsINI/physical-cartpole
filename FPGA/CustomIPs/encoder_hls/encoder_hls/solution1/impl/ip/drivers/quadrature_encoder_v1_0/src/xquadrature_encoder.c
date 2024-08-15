// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xquadrature_encoder.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XQuadrature_encoder_CfgInitialize(XQuadrature_encoder *InstancePtr, XQuadrature_encoder_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Encoder_axi_BaseAddress = ConfigPtr->Encoder_axi_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XQuadrature_encoder_Set_reset(XQuadrature_encoder *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XQuadrature_encoder_WriteReg(InstancePtr->Encoder_axi_BaseAddress, XQUADRATURE_ENCODER_ENCODER_AXI_ADDR_RESET_DATA, Data);
}

u32 XQuadrature_encoder_Get_reset(XQuadrature_encoder *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XQuadrature_encoder_ReadReg(InstancePtr->Encoder_axi_BaseAddress, XQUADRATURE_ENCODER_ENCODER_AXI_ADDR_RESET_DATA);
    return Data;
}

u32 XQuadrature_encoder_Get_count(XQuadrature_encoder *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XQuadrature_encoder_ReadReg(InstancePtr->Encoder_axi_BaseAddress, XQUADRATURE_ENCODER_ENCODER_AXI_ADDR_COUNT_DATA);
    return Data;
}

u32 XQuadrature_encoder_Get_count_vld(XQuadrature_encoder *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XQuadrature_encoder_ReadReg(InstancePtr->Encoder_axi_BaseAddress, XQUADRATURE_ENCODER_ENCODER_AXI_ADDR_COUNT_CTRL);
    return Data & 0x1;
}

