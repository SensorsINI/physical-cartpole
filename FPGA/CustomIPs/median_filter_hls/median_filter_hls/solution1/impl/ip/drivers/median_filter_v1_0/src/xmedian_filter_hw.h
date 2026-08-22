// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// MEDIAN
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of filtered_i
//        bit 31~0 - filtered_i[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of filtered_o
//        bit 31~0 - filtered_o[31:0] (Read)
// 0x1c : Control signal of filtered_o
//        bit 0  - filtered_o_ap_vld (Read/COR)
//        others - reserved
// 0x20 : Data signal of raw_i
//        bit 31~0 - raw_i[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of raw_o
//        bit 31~0 - raw_o[31:0] (Read)
// 0x2c : Control signal of raw_o
//        bit 0  - raw_o_ap_vld (Read/COR)
//        others - reserved
// 0x30 : Data signal of window_size
//        bit 31~0 - window_size[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of trim_count
//        bit 31~0 - trim_count[31:0] (Read/Write)
// 0x3c : reserved
// 0x40 : Data signal of filter_mode
//        bit 31~0 - filter_mode[31:0] (Read/Write)
// 0x44 : reserved
// 0x48 : Data signal of rail_low
//        bit 31~0 - rail_low[31:0] (Read/Write)
// 0x4c : reserved
// 0x50 : Data signal of rail_high
//        bit 31~0 - rail_high[31:0] (Read/Write)
// 0x54 : reserved
// 0x58 : Data signal of dz_status_i
//        bit 31~0 - dz_status_i[31:0] (Read/Write)
// 0x5c : reserved
// 0x60 : Data signal of dz_status_o
//        bit 31~0 - dz_status_o[31:0] (Read)
// 0x64 : Control signal of dz_status_o
//        bit 0  - dz_status_o_ap_vld (Read/COR)
//        others - reserved
// 0x68 : Data signal of dz_window_i
//        bit 31~0 - dz_window_i[31:0] (Read/Write)
// 0x6c : reserved
// 0x70 : Data signal of dz_window_o
//        bit 31~0 - dz_window_o[31:0] (Read)
// 0x74 : Control signal of dz_window_o
//        bit 0  - dz_window_o_ap_vld (Read/COR)
//        others - reserved
// 0x78 : Data signal of dz_age_i
//        bit 31~0 - dz_age_i[31:0] (Read/Write)
// 0x7c : reserved
// 0x80 : Data signal of dz_age_o
//        bit 31~0 - dz_age_o[31:0] (Read)
// 0x84 : Control signal of dz_age_o
//        bit 0  - dz_age_o_ap_vld (Read/COR)
//        others - reserved
// 0x88 : Data signal of dz_low_count
//        bit 31~0 - dz_low_count[31:0] (Read)
// 0x8c : Control signal of dz_low_count
//        bit 0  - dz_low_count_ap_vld (Read/COR)
//        others - reserved
// 0x98 : Data signal of dz_high_count
//        bit 31~0 - dz_high_count[31:0] (Read)
// 0x9c : Control signal of dz_high_count
//        bit 0  - dz_high_count_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XMEDIAN_FILTER_MEDIAN_ADDR_FILTERED_I_DATA    0x10
#define XMEDIAN_FILTER_MEDIAN_BITS_FILTERED_I_DATA    32
#define XMEDIAN_FILTER_MEDIAN_ADDR_FILTERED_O_DATA    0x18
#define XMEDIAN_FILTER_MEDIAN_BITS_FILTERED_O_DATA    32
#define XMEDIAN_FILTER_MEDIAN_ADDR_FILTERED_O_CTRL    0x1c
#define XMEDIAN_FILTER_MEDIAN_ADDR_RAW_I_DATA         0x20
#define XMEDIAN_FILTER_MEDIAN_BITS_RAW_I_DATA         32
#define XMEDIAN_FILTER_MEDIAN_ADDR_RAW_O_DATA         0x28
#define XMEDIAN_FILTER_MEDIAN_BITS_RAW_O_DATA         32
#define XMEDIAN_FILTER_MEDIAN_ADDR_RAW_O_CTRL         0x2c
#define XMEDIAN_FILTER_MEDIAN_ADDR_WINDOW_SIZE_DATA   0x30
#define XMEDIAN_FILTER_MEDIAN_BITS_WINDOW_SIZE_DATA   32
#define XMEDIAN_FILTER_MEDIAN_ADDR_TRIM_COUNT_DATA    0x38
#define XMEDIAN_FILTER_MEDIAN_BITS_TRIM_COUNT_DATA    32
#define XMEDIAN_FILTER_MEDIAN_ADDR_FILTER_MODE_DATA   0x40
#define XMEDIAN_FILTER_MEDIAN_BITS_FILTER_MODE_DATA   32
#define XMEDIAN_FILTER_MEDIAN_ADDR_RAIL_LOW_DATA      0x48
#define XMEDIAN_FILTER_MEDIAN_BITS_RAIL_LOW_DATA      32
#define XMEDIAN_FILTER_MEDIAN_ADDR_RAIL_HIGH_DATA     0x50
#define XMEDIAN_FILTER_MEDIAN_BITS_RAIL_HIGH_DATA     32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_STATUS_I_DATA   0x58
#define XMEDIAN_FILTER_MEDIAN_BITS_DZ_STATUS_I_DATA   32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_STATUS_O_DATA   0x60
#define XMEDIAN_FILTER_MEDIAN_BITS_DZ_STATUS_O_DATA   32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_STATUS_O_CTRL   0x64
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_WINDOW_I_DATA   0x68
#define XMEDIAN_FILTER_MEDIAN_BITS_DZ_WINDOW_I_DATA   32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_WINDOW_O_DATA   0x70
#define XMEDIAN_FILTER_MEDIAN_BITS_DZ_WINDOW_O_DATA   32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_WINDOW_O_CTRL   0x74
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_AGE_I_DATA      0x78
#define XMEDIAN_FILTER_MEDIAN_BITS_DZ_AGE_I_DATA      32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_AGE_O_DATA      0x80
#define XMEDIAN_FILTER_MEDIAN_BITS_DZ_AGE_O_DATA      32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_AGE_O_CTRL      0x84
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_LOW_COUNT_DATA  0x88
#define XMEDIAN_FILTER_MEDIAN_BITS_DZ_LOW_COUNT_DATA  32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_LOW_COUNT_CTRL  0x8c
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_HIGH_COUNT_DATA 0x98
#define XMEDIAN_FILTER_MEDIAN_BITS_DZ_HIGH_COUNT_DATA 32
#define XMEDIAN_FILTER_MEDIAN_ADDR_DZ_HIGH_COUNT_CTRL 0x9c

