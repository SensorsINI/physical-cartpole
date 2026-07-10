/*
 * SecLoc shell: AXI-Lite register map to AXI-Stream packet bridge.
 *
 * No SecLoc or network logic — marshals scalars into the gate request packet
 * and copies the gate response back to output registers.
 */

#include "secloc_shell.h"

static void write_request_packet(
    hls::stream<secloc_axis32_t>& gate_req,
    unsigned control_flags,
    int tick,
    float log_base,
    int ref_period_ticks,
    float dead_ang,
    float dead_pos,
    float angle,
    float angleD,
    float angle_cos,
    float angle_sin,
    float position,
    float positionD,
    float target_equilibrium,
    float target_position)
{
    ap_uint<32> words[SECLOC_REQ_WORDS];
    words[SECLOC_REQ_CONTROL_FLAGS] = control_flags;
    words[SECLOC_REQ_TICK]          = (ap_uint<32>)tick;
    words[SECLOC_REQ_LOG_BASE]      = secloc_float_to_bits(log_base);
    words[SECLOC_REQ_REF_PERIOD]    = (ap_uint<32>)ref_period_ticks;
    words[SECLOC_REQ_DEAD_ANG]      = secloc_float_to_bits(dead_ang);
    words[SECLOC_REQ_DEAD_POS]      = secloc_float_to_bits(dead_pos);
    words[SECLOC_REQ_ANGLE]         = secloc_float_to_bits(angle);
    words[SECLOC_REQ_ANGLED]        = secloc_float_to_bits(angleD);
    words[SECLOC_REQ_ANGLE_COS]     = secloc_float_to_bits(angle_cos);
    words[SECLOC_REQ_ANGLE_SIN]     = secloc_float_to_bits(angle_sin);
    words[SECLOC_REQ_POSITION]      = secloc_float_to_bits(position);
    words[SECLOC_REQ_POSITIOND]     = secloc_float_to_bits(positionD);
    words[SECLOC_REQ_TARGET_EQ]     = secloc_float_to_bits(target_equilibrium);
    words[SECLOC_REQ_TARGET_POS]    = secloc_float_to_bits(target_position);

WRITE_REQ:
    for (int i = 0; i < SECLOC_REQ_WORDS; ++i) {
#pragma HLS PIPELINE II=1
        bool last = (i == SECLOC_REQ_WORDS - 1);
        gate_req.write(secloc_make_beat(words[i], last));
    }
}

static void read_response_packet(
    hls::stream<secloc_axis32_t>& gate_resp,
    float* Q,
    unsigned* status,
    unsigned* update_count,
    unsigned* nn_wait_cycles)
{
    ap_uint<32> words[SECLOC_RESP_WORDS];
READ_RESP:
    for (int i = 0; i < SECLOC_RESP_WORDS; ++i) {
#pragma HLS PIPELINE II=1
        secloc_axis32_t beat = gate_resp.read();
        words[i] = beat.data;
    }

    *Q              = secloc_bits_to_float(words[SECLOC_RESP_Q]);
    *status         = (unsigned)words[SECLOC_RESP_STATUS];
    *update_count   = (unsigned)words[SECLOC_RESP_UPDATE_COUNT];
    *nn_wait_cycles = (unsigned)words[SECLOC_RESP_NN_WAIT];
}

void secloc_shell(
    float angleD,
    float angle_cos,
    float angle_sin,
    float position,
    float positionD,
    float target_equilibrium,
    float target_position,
    float angle,
    int   tick,
    float log_base,
    int   ref_period_ticks,
    float dead_ang,
    float dead_pos,
    unsigned control_flags,
    float*    Q,
    unsigned* status,
    unsigned* update_count,
    unsigned* nn_wait_cycles,
    hls::stream<secloc_axis32_t>& gate_req,
    hls::stream<secloc_axis32_t>& gate_resp)
{
#pragma HLS INTERFACE s_axilite port=return             bundle=CTRL
#pragma HLS INTERFACE s_axilite port=angleD             bundle=CTRL
#pragma HLS INTERFACE s_axilite port=angle_cos          bundle=CTRL
#pragma HLS INTERFACE s_axilite port=angle_sin          bundle=CTRL
#pragma HLS INTERFACE s_axilite port=position           bundle=CTRL
#pragma HLS INTERFACE s_axilite port=positionD          bundle=CTRL
#pragma HLS INTERFACE s_axilite port=target_equilibrium bundle=CTRL
#pragma HLS INTERFACE s_axilite port=target_position    bundle=CTRL
#pragma HLS INTERFACE s_axilite port=angle              bundle=CTRL
#pragma HLS INTERFACE s_axilite port=tick               bundle=CTRL
#pragma HLS INTERFACE s_axilite port=log_base           bundle=CTRL
#pragma HLS INTERFACE s_axilite port=ref_period_ticks   bundle=CTRL
#pragma HLS INTERFACE s_axilite port=dead_ang           bundle=CTRL
#pragma HLS INTERFACE s_axilite port=dead_pos           bundle=CTRL
#pragma HLS INTERFACE s_axilite port=control_flags      bundle=CTRL
#pragma HLS INTERFACE s_axilite port=Q                  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=status             bundle=CTRL
#pragma HLS INTERFACE s_axilite port=update_count       bundle=CTRL
#pragma HLS INTERFACE s_axilite port=nn_wait_cycles     bundle=CTRL
#pragma HLS INTERFACE axis port=gate_req
#pragma HLS INTERFACE axis port=gate_resp

    write_request_packet(
        gate_req, control_flags, tick,
        log_base, ref_period_ticks, dead_ang, dead_pos,
        angle, angleD, angle_cos, angle_sin,
        position, positionD, target_equilibrium, target_position);

    read_response_packet(gate_resp, Q, status, update_count, nn_wait_cycles);
}

void secloc_shell_write_transaction(
    float angleD, float angle_cos, float angle_sin,
    float position, float positionD,
    float target_equilibrium, float target_position,
    float angle, int tick,
    float log_base, int ref_period_ticks,
    float dead_ang, float dead_pos,
    unsigned control_flags,
    hls::stream<secloc_axis32_t>& gate_req)
{
    write_request_packet(
        gate_req, control_flags, tick,
        log_base, ref_period_ticks, dead_ang, dead_pos,
        angle, angleD, angle_cos, angle_sin,
        position, positionD, target_equilibrium, target_position);
}

void secloc_shell_read_transaction(
    hls::stream<secloc_axis32_t>& gate_resp,
    float* Q, unsigned* status,
    unsigned* update_count, unsigned* nn_wait_cycles)
{
    read_response_packet(gate_resp, Q, status, update_count, nn_wait_cycles);
}
