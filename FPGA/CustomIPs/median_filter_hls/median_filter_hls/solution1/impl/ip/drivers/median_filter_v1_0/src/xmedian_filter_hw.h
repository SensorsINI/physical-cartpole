// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// MEDIAN
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of median_i
//        bit 31~0 - median_i[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of median_o
//        bit 31~0 - median_o[31:0] (Read)
// 0x1c : Control signal of median_o
//        bit 0  - median_o_ap_vld (Read/COR)
//        others - reserved
// 0x20 : Data signal of window_size
//        bit 31~0 - window_size[31:0] (Read/Write)
// 0x24 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XMEDIAN_FILTER_MEDIAN_ADDR_MEDIAN_I_DATA    0x10
#define XMEDIAN_FILTER_MEDIAN_BITS_MEDIAN_I_DATA    32
#define XMEDIAN_FILTER_MEDIAN_ADDR_MEDIAN_O_DATA    0x18
#define XMEDIAN_FILTER_MEDIAN_BITS_MEDIAN_O_DATA    32
#define XMEDIAN_FILTER_MEDIAN_ADDR_MEDIAN_O_CTRL    0x1c
#define XMEDIAN_FILTER_MEDIAN_ADDR_WINDOW_SIZE_DATA 0x20
#define XMEDIAN_FILTER_MEDIAN_BITS_WINDOW_SIZE_DATA 32

