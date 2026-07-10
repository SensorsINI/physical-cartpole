// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xsecloc_shell.h"

extern XSecloc_shell_Config XSecloc_shell_ConfigTable[];

XSecloc_shell_Config *XSecloc_shell_LookupConfig(u16 DeviceId) {
	XSecloc_shell_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XSECLOC_SHELL_NUM_INSTANCES; Index++) {
		if (XSecloc_shell_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XSecloc_shell_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XSecloc_shell_Initialize(XSecloc_shell *InstancePtr, u16 DeviceId) {
	XSecloc_shell_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XSecloc_shell_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XSecloc_shell_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

