#ifndef HW_PLATFORM_CONFIG_H
#define HW_PLATFORM_CONFIG_H

/**
 * @file hw_platform_config.h
 * @brief Central hardware platform configuration mapping
 * 
 * This file maps hardware-specific XPAR definitions to logical names used
 * throughout the firmware. When hardware changes (entity names, addresses, etc.),
 * only this file needs to be updated.
 * 
 * The firmware code should use the logical HW_* definitions, not XPAR_* directly.
 */

#include "xparameters.h"

/* ========================================================================
 * Neural Network Accelerator Interfaces
 * ======================================================================== */

/* Controller AXI (formerly MLP_AXI_INTERFACE) - AXI4 Memory-Mapped Interface */
#ifdef XPAR_HARDWARE_ACCEL_CONTROLLER_AXI_0_BASEADDR
    #define HW_CONTROLLER_AXI_BASEADDR  XPAR_HARDWARE_ACCEL_CONTROLLER_AXI_0_BASEADDR
    #define HW_HAS_CONTROLLER_AXI       1
#else
    #define HW_HAS_CONTROLLER_AXI       0
#endif

/* Controller AXI-Lite Interface */
#ifdef XPAR_HARDWARE_ACCEL_CONTROLLER_AXILITE_0_BASEADDR
    #define HW_CONTROLLER_AXILITE_BASEADDR  XPAR_HARDWARE_ACCEL_CONTROLLER_AXILITE_0_BASEADDR
    #define HW_HAS_CONTROLLER_AXILITE       1
#else
    #define HW_HAS_CONTROLLER_AXILITE       0
#endif

/* HLS4ML AXI-Stream DMA Interface */
#ifdef XPAR_HARDWARE_ACCEL_HLS4ML_AXI_DMA_0_DEVICE_ID
    #define HW_HLS4ML_DMA_DEVICE_ID     XPAR_HARDWARE_ACCEL_HLS4ML_AXI_DMA_0_DEVICE_ID
    #define HW_HAS_HLS4ML_DMA           1
#else
    #define HW_HAS_HLS4ML_DMA           0
#endif

/* EdgeDRNN AXI-Stream DMA Interface */
#ifdef XPAR_HARDWARE_ACCEL_EDGEDRNN_AXI_DMA_1_DEVICE_ID
    #define HW_EDGEDRNN_DMA_DEVICE_ID   XPAR_HARDWARE_ACCEL_EDGEDRNN_AXI_DMA_1_DEVICE_ID
    #define HW_HAS_EDGEDRNN_DMA         1
#else
    #define HW_HAS_EDGEDRNN_DMA         0
#endif

/* DiffLogic AXI-Stream DMA Interface */
#ifdef XPAR_HARDWARE_ACCEL_DIFFLG_AXI_DMA_0_DEVICE_ID
    #define HW_DIFFLG_DMA_DEVICE_ID     XPAR_HARDWARE_ACCEL_DIFFLG_AXI_DMA_0_DEVICE_ID
    #define HW_HAS_DIFFLG_DMA           1
#else
    #define HW_HAS_DIFFLG_DMA           0
#endif

/* ========================================================================
 * AXI CDMA for Burst Transfers
 * ======================================================================== */

#ifdef XPAR_HARDWARE_ACCEL_AXI_CDMA_0_DEVICE_ID
    #define HW_CDMA_DEVICE_ID           XPAR_HARDWARE_ACCEL_AXI_CDMA_0_DEVICE_ID
    #define HW_HAS_CDMA                 1
#else
    #define HW_HAS_CDMA                 0
#endif

/* ========================================================================
 * Feature Flags - Which accelerators are present?
 * ======================================================================== */

/* HLS4ML is available if ANY of its interfaces are present */
#if HW_HAS_CONTROLLER_AXI || HW_HAS_CONTROLLER_AXILITE || HW_HAS_HLS4ML_DMA
    #define HW_HAS_HLS4ML               1
#else
    #define HW_HAS_HLS4ML               0
#endif

/* EdgeDRNN is available if its DMA interface is present */
#define HW_HAS_EDGEDRNN                 HW_HAS_EDGEDRNN_DMA

/* DiffLogic is available if its DMA interface is present */
#define HW_HAS_DIFFLG                   HW_HAS_DIFFLG_DMA

#endif /* HW_PLATFORM_CONFIG_H */

