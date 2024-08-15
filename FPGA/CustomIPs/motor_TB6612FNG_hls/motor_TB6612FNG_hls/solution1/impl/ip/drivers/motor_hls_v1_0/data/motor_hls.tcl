# ==============================================================
# Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
# Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
# ==============================================================
proc generate {drv_handle} {
    xdefine_include_file $drv_handle "xparameters.h" "XMotor_hls" \
        "NUM_INSTANCES" \
        "DEVICE_ID" \
        "C_S_AXI_MOTOR_AXI_BASEADDR" \
        "C_S_AXI_MOTOR_AXI_HIGHADDR"

    xdefine_config_file $drv_handle "xmotor_hls_g.c" "XMotor_hls" \
        "DEVICE_ID" \
        "C_S_AXI_MOTOR_AXI_BASEADDR"

    xdefine_canonical_xpars $drv_handle "xparameters.h" "XMotor_hls" \
        "DEVICE_ID" \
        "C_S_AXI_MOTOR_AXI_BASEADDR" \
        "C_S_AXI_MOTOR_AXI_HIGHADDR"
}

