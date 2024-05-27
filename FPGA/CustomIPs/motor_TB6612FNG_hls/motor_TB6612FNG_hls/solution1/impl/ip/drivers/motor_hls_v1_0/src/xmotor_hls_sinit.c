// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xmotor_hls.h"

extern XMotor_hls_Config XMotor_hls_ConfigTable[];

XMotor_hls_Config *XMotor_hls_LookupConfig(u16 DeviceId) {
	XMotor_hls_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XMOTOR_HLS_NUM_INSTANCES; Index++) {
		if (XMotor_hls_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XMotor_hls_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XMotor_hls_Initialize(XMotor_hls *InstancePtr, u16 DeviceId) {
	XMotor_hls_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XMotor_hls_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XMotor_hls_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

