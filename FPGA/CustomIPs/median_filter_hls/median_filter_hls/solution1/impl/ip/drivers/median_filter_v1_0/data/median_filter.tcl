# ==============================================================
# Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
# Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
# ==============================================================
proc generate {drv_handle} {
    xdefine_include_file $drv_handle "xparameters.h" "XMedian_filter" \
        "NUM_INSTANCES" \
        "DEVICE_ID" \
        "C_S_AXI_MEDIAN_BASEADDR" \
        "C_S_AXI_MEDIAN_HIGHADDR"

    xdefine_config_file $drv_handle "xmedian_filter_g.c" "XMedian_filter" \
        "DEVICE_ID" \
        "C_S_AXI_MEDIAN_BASEADDR"

    xdefine_canonical_xpars $drv_handle "xparameters.h" "XMedian_filter" \
        "DEVICE_ID" \
        "C_S_AXI_MEDIAN_BASEADDR" \
        "C_S_AXI_MEDIAN_HIGHADDR"
}

