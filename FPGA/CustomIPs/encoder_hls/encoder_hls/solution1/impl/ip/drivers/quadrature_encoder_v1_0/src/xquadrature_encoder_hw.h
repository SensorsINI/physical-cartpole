// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// ENCODER_AXI
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of reset
//        bit 0  - reset[0] (Read/Write)
//        others - reserved
// 0x14 : reserved
// 0x18 : Data signal of count
//        bit 31~0 - count[31:0] (Read)
// 0x1c : Control signal of count
//        bit 0  - count_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XQUADRATURE_ENCODER_ENCODER_AXI_ADDR_RESET_DATA 0x10
#define XQUADRATURE_ENCODER_ENCODER_AXI_BITS_RESET_DATA 1
#define XQUADRATURE_ENCODER_ENCODER_AXI_ADDR_COUNT_DATA 0x18
#define XQUADRATURE_ENCODER_ENCODER_AXI_BITS_COUNT_DATA 32
#define XQUADRATURE_ENCODER_ENCODER_AXI_ADDR_COUNT_CTRL 0x1c

