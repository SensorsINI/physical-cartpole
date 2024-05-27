// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xquadrature_encoder.h"

extern XQuadrature_encoder_Config XQuadrature_encoder_ConfigTable[];

XQuadrature_encoder_Config *XQuadrature_encoder_LookupConfig(u16 DeviceId) {
	XQuadrature_encoder_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XQUADRATURE_ENCODER_NUM_INSTANCES; Index++) {
		if (XQuadrature_encoder_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XQuadrature_encoder_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XQuadrature_encoder_Initialize(XQuadrature_encoder *InstancePtr, u16 DeviceId) {
	XQuadrature_encoder_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XQuadrature_encoder_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XQuadrature_encoder_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

