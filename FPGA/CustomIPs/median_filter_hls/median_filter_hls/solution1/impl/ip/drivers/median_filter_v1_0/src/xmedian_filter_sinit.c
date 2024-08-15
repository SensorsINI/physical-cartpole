// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xmedian_filter.h"

extern XMedian_filter_Config XMedian_filter_ConfigTable[];

XMedian_filter_Config *XMedian_filter_LookupConfig(u16 DeviceId) {
	XMedian_filter_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XMEDIAN_FILTER_NUM_INSTANCES; Index++) {
		if (XMedian_filter_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XMedian_filter_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XMedian_filter_Initialize(XMedian_filter *InstancePtr, u16 DeviceId) {
	XMedian_filter_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XMedian_filter_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XMedian_filter_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

