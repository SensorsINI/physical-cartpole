// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xsecloc_shell.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XSecloc_shell_CfgInitialize(XSecloc_shell *InstancePtr, XSecloc_shell_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Ctrl_BaseAddress = ConfigPtr->Ctrl_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XSecloc_shell_Start(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_AP_CTRL) & 0x80;
    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XSecloc_shell_IsDone(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XSecloc_shell_IsIdle(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XSecloc_shell_IsReady(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XSecloc_shell_EnableAutoRestart(XSecloc_shell *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_AP_CTRL, 0x80);
}

void XSecloc_shell_DisableAutoRestart(XSecloc_shell *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_AP_CTRL, 0);
}

void XSecloc_shell_Set_angleD(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ANGLED_DATA, Data);
}

u32 XSecloc_shell_Get_angleD(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ANGLED_DATA);
    return Data;
}

void XSecloc_shell_Set_angle_cos(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ANGLE_COS_DATA, Data);
}

u32 XSecloc_shell_Get_angle_cos(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ANGLE_COS_DATA);
    return Data;
}

void XSecloc_shell_Set_angle_sin(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ANGLE_SIN_DATA, Data);
}

u32 XSecloc_shell_Get_angle_sin(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ANGLE_SIN_DATA);
    return Data;
}

void XSecloc_shell_Set_position(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_POSITION_DATA, Data);
}

u32 XSecloc_shell_Get_position(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_POSITION_DATA);
    return Data;
}

void XSecloc_shell_Set_positionD(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_POSITIOND_DATA, Data);
}

u32 XSecloc_shell_Get_positionD(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_POSITIOND_DATA);
    return Data;
}

void XSecloc_shell_Set_target_equilibrium(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_TARGET_EQUILIBRIUM_DATA, Data);
}

u32 XSecloc_shell_Get_target_equilibrium(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_TARGET_EQUILIBRIUM_DATA);
    return Data;
}

void XSecloc_shell_Set_target_position(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_TARGET_POSITION_DATA, Data);
}

u32 XSecloc_shell_Get_target_position(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_TARGET_POSITION_DATA);
    return Data;
}

void XSecloc_shell_Set_angle(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ANGLE_DATA, Data);
}

u32 XSecloc_shell_Get_angle(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ANGLE_DATA);
    return Data;
}

void XSecloc_shell_Set_tick(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_TICK_DATA, Data);
}

u32 XSecloc_shell_Get_tick(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_TICK_DATA);
    return Data;
}

void XSecloc_shell_Set_log_base(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_LOG_BASE_DATA, Data);
}

u32 XSecloc_shell_Get_log_base(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_LOG_BASE_DATA);
    return Data;
}

void XSecloc_shell_Set_ref_period_ticks(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_REF_PERIOD_TICKS_DATA, Data);
}

u32 XSecloc_shell_Get_ref_period_ticks(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_REF_PERIOD_TICKS_DATA);
    return Data;
}

void XSecloc_shell_Set_dead_ang(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_DEAD_ANG_DATA, Data);
}

u32 XSecloc_shell_Get_dead_ang(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_DEAD_ANG_DATA);
    return Data;
}

void XSecloc_shell_Set_dead_pos(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_DEAD_POS_DATA, Data);
}

u32 XSecloc_shell_Get_dead_pos(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_DEAD_POS_DATA);
    return Data;
}

void XSecloc_shell_Set_control_flags(XSecloc_shell *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_CONTROL_FLAGS_DATA, Data);
}

u32 XSecloc_shell_Get_control_flags(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_CONTROL_FLAGS_DATA);
    return Data;
}

u32 XSecloc_shell_Get_Q(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_Q_DATA);
    return Data;
}

u32 XSecloc_shell_Get_Q_vld(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_Q_CTRL);
    return Data & 0x1;
}

u32 XSecloc_shell_Get_status(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_STATUS_DATA);
    return Data;
}

u32 XSecloc_shell_Get_status_vld(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_STATUS_CTRL);
    return Data & 0x1;
}

u32 XSecloc_shell_Get_update_count(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_UPDATE_COUNT_DATA);
    return Data;
}

u32 XSecloc_shell_Get_update_count_vld(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_UPDATE_COUNT_CTRL);
    return Data & 0x1;
}

u32 XSecloc_shell_Get_nn_wait_cycles(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_NN_WAIT_CYCLES_DATA);
    return Data;
}

u32 XSecloc_shell_Get_nn_wait_cycles_vld(XSecloc_shell *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_NN_WAIT_CYCLES_CTRL);
    return Data & 0x1;
}

void XSecloc_shell_InterruptGlobalEnable(XSecloc_shell *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_GIE, 1);
}

void XSecloc_shell_InterruptGlobalDisable(XSecloc_shell *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_GIE, 0);
}

void XSecloc_shell_InterruptEnable(XSecloc_shell *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_IER);
    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_IER, Register | Mask);
}

void XSecloc_shell_InterruptDisable(XSecloc_shell *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_IER);
    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_IER, Register & (~Mask));
}

void XSecloc_shell_InterruptClear(XSecloc_shell *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSecloc_shell_WriteReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ISR, Mask);
}

u32 XSecloc_shell_InterruptGetEnabled(XSecloc_shell *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_IER);
}

u32 XSecloc_shell_InterruptGetStatus(XSecloc_shell *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XSecloc_shell_ReadReg(InstancePtr->Ctrl_BaseAddress, XSECLOC_SHELL_CTRL_ADDR_ISR);
}

