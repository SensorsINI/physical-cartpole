// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// CTRL
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read)
//        bit 7  - auto_restart (Read/Write)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0  - enable ap_done interrupt (Read/Write)
//        bit 1  - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0  - ap_done (COR/TOW)
//        bit 1  - ap_ready (COR/TOW)
//        others - reserved
// 0x10 : Data signal of angleD
//        bit 31~0 - angleD[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of angle_cos
//        bit 31~0 - angle_cos[31:0] (Read/Write)
// 0x1c : reserved
// 0x20 : Data signal of angle_sin
//        bit 31~0 - angle_sin[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of position
//        bit 31~0 - position[31:0] (Read/Write)
// 0x2c : reserved
// 0x30 : Data signal of positionD
//        bit 31~0 - positionD[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of target_equilibrium
//        bit 31~0 - target_equilibrium[31:0] (Read/Write)
// 0x3c : reserved
// 0x40 : Data signal of target_position
//        bit 31~0 - target_position[31:0] (Read/Write)
// 0x44 : reserved
// 0x48 : Data signal of angle
//        bit 31~0 - angle[31:0] (Read/Write)
// 0x4c : reserved
// 0x50 : Data signal of tick
//        bit 31~0 - tick[31:0] (Read/Write)
// 0x54 : reserved
// 0x58 : Data signal of log_base
//        bit 31~0 - log_base[31:0] (Read/Write)
// 0x5c : reserved
// 0x60 : Data signal of ref_period_ticks
//        bit 31~0 - ref_period_ticks[31:0] (Read/Write)
// 0x64 : reserved
// 0x68 : Data signal of dead_ang
//        bit 31~0 - dead_ang[31:0] (Read/Write)
// 0x6c : reserved
// 0x70 : Data signal of dead_pos
//        bit 31~0 - dead_pos[31:0] (Read/Write)
// 0x74 : reserved
// 0x78 : Data signal of control_flags
//        bit 31~0 - control_flags[31:0] (Read/Write)
// 0x7c : reserved
// 0x80 : Data signal of Q
//        bit 31~0 - Q[31:0] (Read)
// 0x84 : Control signal of Q
//        bit 0  - Q_ap_vld (Read/COR)
//        others - reserved
// 0x90 : Data signal of status
//        bit 31~0 - status[31:0] (Read)
// 0x94 : Control signal of status
//        bit 0  - status_ap_vld (Read/COR)
//        others - reserved
// 0xa0 : Data signal of update_count
//        bit 31~0 - update_count[31:0] (Read)
// 0xa4 : Control signal of update_count
//        bit 0  - update_count_ap_vld (Read/COR)
//        others - reserved
// 0xb0 : Data signal of nn_wait_cycles
//        bit 31~0 - nn_wait_cycles[31:0] (Read)
// 0xb4 : Control signal of nn_wait_cycles
//        bit 0  - nn_wait_cycles_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XSECLOC_SHELL_CTRL_ADDR_AP_CTRL                 0x00
#define XSECLOC_SHELL_CTRL_ADDR_GIE                     0x04
#define XSECLOC_SHELL_CTRL_ADDR_IER                     0x08
#define XSECLOC_SHELL_CTRL_ADDR_ISR                     0x0c
#define XSECLOC_SHELL_CTRL_ADDR_ANGLED_DATA             0x10
#define XSECLOC_SHELL_CTRL_BITS_ANGLED_DATA             32
#define XSECLOC_SHELL_CTRL_ADDR_ANGLE_COS_DATA          0x18
#define XSECLOC_SHELL_CTRL_BITS_ANGLE_COS_DATA          32
#define XSECLOC_SHELL_CTRL_ADDR_ANGLE_SIN_DATA          0x20
#define XSECLOC_SHELL_CTRL_BITS_ANGLE_SIN_DATA          32
#define XSECLOC_SHELL_CTRL_ADDR_POSITION_DATA           0x28
#define XSECLOC_SHELL_CTRL_BITS_POSITION_DATA           32
#define XSECLOC_SHELL_CTRL_ADDR_POSITIOND_DATA          0x30
#define XSECLOC_SHELL_CTRL_BITS_POSITIOND_DATA          32
#define XSECLOC_SHELL_CTRL_ADDR_TARGET_EQUILIBRIUM_DATA 0x38
#define XSECLOC_SHELL_CTRL_BITS_TARGET_EQUILIBRIUM_DATA 32
#define XSECLOC_SHELL_CTRL_ADDR_TARGET_POSITION_DATA    0x40
#define XSECLOC_SHELL_CTRL_BITS_TARGET_POSITION_DATA    32
#define XSECLOC_SHELL_CTRL_ADDR_ANGLE_DATA              0x48
#define XSECLOC_SHELL_CTRL_BITS_ANGLE_DATA              32
#define XSECLOC_SHELL_CTRL_ADDR_TICK_DATA               0x50
#define XSECLOC_SHELL_CTRL_BITS_TICK_DATA               32
#define XSECLOC_SHELL_CTRL_ADDR_LOG_BASE_DATA           0x58
#define XSECLOC_SHELL_CTRL_BITS_LOG_BASE_DATA           32
#define XSECLOC_SHELL_CTRL_ADDR_REF_PERIOD_TICKS_DATA   0x60
#define XSECLOC_SHELL_CTRL_BITS_REF_PERIOD_TICKS_DATA   32
#define XSECLOC_SHELL_CTRL_ADDR_DEAD_ANG_DATA           0x68
#define XSECLOC_SHELL_CTRL_BITS_DEAD_ANG_DATA           32
#define XSECLOC_SHELL_CTRL_ADDR_DEAD_POS_DATA           0x70
#define XSECLOC_SHELL_CTRL_BITS_DEAD_POS_DATA           32
#define XSECLOC_SHELL_CTRL_ADDR_CONTROL_FLAGS_DATA      0x78
#define XSECLOC_SHELL_CTRL_BITS_CONTROL_FLAGS_DATA      32
#define XSECLOC_SHELL_CTRL_ADDR_Q_DATA                  0x80
#define XSECLOC_SHELL_CTRL_BITS_Q_DATA                  32
#define XSECLOC_SHELL_CTRL_ADDR_Q_CTRL                  0x84
#define XSECLOC_SHELL_CTRL_ADDR_STATUS_DATA             0x90
#define XSECLOC_SHELL_CTRL_BITS_STATUS_DATA             32
#define XSECLOC_SHELL_CTRL_ADDR_STATUS_CTRL             0x94
#define XSECLOC_SHELL_CTRL_ADDR_UPDATE_COUNT_DATA       0xa0
#define XSECLOC_SHELL_CTRL_BITS_UPDATE_COUNT_DATA       32
#define XSECLOC_SHELL_CTRL_ADDR_UPDATE_COUNT_CTRL       0xa4
#define XSECLOC_SHELL_CTRL_ADDR_NN_WAIT_CYCLES_DATA     0xb0
#define XSECLOC_SHELL_CTRL_BITS_NN_WAIT_CYCLES_DATA     32
#define XSECLOC_SHELL_CTRL_ADDR_NN_WAIT_CYCLES_CTRL     0xb4

