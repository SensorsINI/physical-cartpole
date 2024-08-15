// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// MOTOR_AXI
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of pwm_period_in_clock_cycles
//        bit 31~0 - pwm_period_in_clock_cycles[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of pwm_duty_cycle_in_clock_cycles
//        bit 31~0 - pwm_duty_cycle_in_clock_cycles[31:0] (Read/Write)
// 0x1c : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XMOTOR_HLS_MOTOR_AXI_ADDR_PWM_PERIOD_IN_CLOCK_CYCLES_DATA     0x10
#define XMOTOR_HLS_MOTOR_AXI_BITS_PWM_PERIOD_IN_CLOCK_CYCLES_DATA     32
#define XMOTOR_HLS_MOTOR_AXI_ADDR_PWM_DUTY_CYCLE_IN_CLOCK_CYCLES_DATA 0x18
#define XMOTOR_HLS_MOTOR_AXI_BITS_PWM_DUTY_CYCLE_IN_CLOCK_CYCLES_DATA 32

