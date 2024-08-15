// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xmotor_hls.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XMotor_hls_CfgInitialize(XMotor_hls *InstancePtr, XMotor_hls_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Motor_axi_BaseAddress = ConfigPtr->Motor_axi_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XMotor_hls_Set_pwm_period_in_clock_cycles(XMotor_hls *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMotor_hls_WriteReg(InstancePtr->Motor_axi_BaseAddress, XMOTOR_HLS_MOTOR_AXI_ADDR_PWM_PERIOD_IN_CLOCK_CYCLES_DATA, Data);
}

u32 XMotor_hls_Get_pwm_period_in_clock_cycles(XMotor_hls *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMotor_hls_ReadReg(InstancePtr->Motor_axi_BaseAddress, XMOTOR_HLS_MOTOR_AXI_ADDR_PWM_PERIOD_IN_CLOCK_CYCLES_DATA);
    return Data;
}

void XMotor_hls_Set_pwm_duty_cycle_in_clock_cycles(XMotor_hls *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMotor_hls_WriteReg(InstancePtr->Motor_axi_BaseAddress, XMOTOR_HLS_MOTOR_AXI_ADDR_PWM_DUTY_CYCLE_IN_CLOCK_CYCLES_DATA, Data);
}

u32 XMotor_hls_Get_pwm_duty_cycle_in_clock_cycles(XMotor_hls *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMotor_hls_ReadReg(InstancePtr->Motor_axi_BaseAddress, XMOTOR_HLS_MOTOR_AXI_ADDR_PWM_DUTY_CYCLE_IN_CLOCK_CYCLES_DATA);
    return Data;
}

