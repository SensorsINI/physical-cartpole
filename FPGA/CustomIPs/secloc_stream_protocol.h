#ifndef SECLOC_STREAM_PROTOCOL_H
#define SECLOC_STREAM_PROTOCOL_H

/*
 * Shared AXI-Stream packet layout for the three-block SecLoc chain:
 *   secloc_shell (AXI-Lite) <-> secloc_gate <-> nn_marshal <-> controller_axis
 *
 * All beats use ap_axiu<32,0,0,0> (TDATA + TLAST; KEEP/STRB tied to all-ones
 * where required by downstream slaves).
 */

#include <ap_int.h>
#include <ap_axi_sdata.h>
#include <cstring>

#define SECLOC_STREAM_VERSION 2u

/* Cartpole NN input count on the gate<->marshal link (raw float32 beats).
 * Must equal NN_IO_INPUT_COUNT in the deployed network's nn_marshal_config.h. */
#define SECLOC_NN_INPUTS     7

/* control_flags bits (shell request word 0) */
#define SECLOC_CTRL_GATE_RESET     0x1u
#define SECLOC_CTRL_FORCE_COMPUTE  0x2u

/* status bits; bits [31:24] carry SECLOC_STREAM_VERSION */
#define SECLOC_STATUS_FIRED           0x1u
#define SECLOC_STATUS_GATE_EVALUATED  0x2u
#define SECLOC_STATUS_SKIPPED         0x4u
#define SECLOC_STATUS_NN_TIMEOUT      0x8u

/* Bounded wait for marshal/network response (~40x expected inference). */
#define SECLOC_NN_TIMEOUT_CYCLES 4096u

typedef ap_axiu<32, 0, 0, 0> secloc_axis32_t;

/* Shell -> gate request packet (14 words, TLAST on last) */
#define SECLOC_REQ_CONTROL_FLAGS  0
#define SECLOC_REQ_TICK           1
#define SECLOC_REQ_LOG_BASE       2
#define SECLOC_REQ_REF_PERIOD     3
#define SECLOC_REQ_DEAD_ANG       4
#define SECLOC_REQ_DEAD_POS       5
#define SECLOC_REQ_ANGLE          6
#define SECLOC_REQ_ANGLED         7
#define SECLOC_REQ_ANGLE_COS      8
#define SECLOC_REQ_ANGLE_SIN      9
#define SECLOC_REQ_POSITION       10
#define SECLOC_REQ_POSITIOND      11
#define SECLOC_REQ_TARGET_EQ      12
#define SECLOC_REQ_TARGET_POS     13
#define SECLOC_REQ_WORDS          14

/* Gate -> shell response packet (4 words, TLAST on last) */
#define SECLOC_RESP_Q             0
#define SECLOC_RESP_STATUS        1
#define SECLOC_RESP_UPDATE_COUNT  2
#define SECLOC_RESP_NN_WAIT       3
#define SECLOC_RESP_WORDS         4

/* Gate -> marshal / marshal -> gate: 7 raw float32 beats (TLAST on last) */
#define SECLOC_MARSHAL_INPUTS     SECLOC_NN_INPUTS

static inline ap_uint<32> secloc_float_to_bits(float value)
{
    ap_uint<32> bits = 0;
    std::memcpy(&bits, &value, sizeof(value));
    return bits;
}

static inline float secloc_bits_to_float(ap_uint<32> bits)
{
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

static inline secloc_axis32_t secloc_make_beat(ap_uint<32> data, bool last)
{
    secloc_axis32_t beat;
    beat.data = data;
    beat.keep = 0xF;
    beat.strb = 0xF;
    beat.last = last ? 1 : 0;
    return beat;
}

#endif /* SECLOC_STREAM_PROTOCOL_H */
