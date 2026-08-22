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

/* SecLoc shell (HLS IP): AXI-Lite register interface for the three-block
 * SecLoc chain (shell + gate + marshal in front of the hls4ml network).
 * Register map: FPGA/CustomIPs/secloc_shell_hls (Vitis-generated
 * xsecloc_shell_hw.h, mirrored in secloc_frontend_link.c). */
#if defined(XPAR_HARDWARE_ACCEL_SECLOC_SHELL_0_S_AXI_CTRL_BASEADDR)
    #define HW_SECLOC_FRONTEND_BASEADDR XPAR_HARDWARE_ACCEL_SECLOC_SHELL_0_S_AXI_CTRL_BASEADDR
    #define HW_HAS_SECLOC_FRONTEND      1
#elif defined(XPAR_XSECLOC_SHELL_0_S_AXI_CTRL_BASEADDR)
    #define HW_SECLOC_FRONTEND_BASEADDR XPAR_XSECLOC_SHELL_0_S_AXI_CTRL_BASEADDR
    #define HW_HAS_SECLOC_FRONTEND      1
#elif defined(XPAR_HARDWARE_ACCEL_SECLOC_FRONTEND_0_S_AXI_CTRL_BASEADDR)
    #define HW_SECLOC_FRONTEND_BASEADDR XPAR_HARDWARE_ACCEL_SECLOC_FRONTEND_0_S_AXI_CTRL_BASEADDR
    #define HW_HAS_SECLOC_FRONTEND      1
#elif defined(XPAR_XSECLOC_FRONTEND_0_S_AXI_CTRL_BASEADDR)
    #define HW_SECLOC_FRONTEND_BASEADDR XPAR_XSECLOC_FRONTEND_0_S_AXI_CTRL_BASEADDR
    #define HW_HAS_SECLOC_FRONTEND      1
#else
    #define HW_HAS_SECLOC_FRONTEND      0
#endif

/* HLS4ML AXI-Stream DMA Interface */
#ifdef XPAR_HARDWARE_ACCEL_CONTROLLER_AXI_DMA_0_DEVICE_ID
    #define HW_HLS4ML_DMA_DEVICE_ID     XPAR_HARDWARE_ACCEL_CONTROLLER_AXI_DMA_0_DEVICE_ID
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

/* HLS4ML is available if ANY of its interfaces are present (the SecLoc
 * frontend reaches the network over its internal AXIS link) */
#if HW_HAS_CONTROLLER_AXI || HW_HAS_CONTROLLER_AXILITE || HW_HAS_HLS4ML_DMA || HW_HAS_SECLOC_FRONTEND
    #define HW_HAS_HLS4ML               1
#else
    #define HW_HAS_HLS4ML               0
#endif

/* EdgeDRNN is available if its DMA interface is present */
#define HW_HAS_EDGEDRNN                 HW_HAS_EDGEDRNN_DMA

/* DiffLogic is available if its DMA interface is present */
#define HW_HAS_DIFFLG                   HW_HAS_DIFFLG_DMA

#endif /* HW_PLATFORM_CONFIG_H */

