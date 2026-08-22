#ifndef SECLOC_SHELL_H
#define SECLOC_SHELL_H

/*
 * SecLoc shell: AXI-Lite slave towards the PS, AXI-Stream towards secloc_gate.
 *
 * Top-function scalar signature matches the retired secloc_frontend IP so the
 * firmware register map and driver stay unchanged.
 */

#include <hls_stream.h>
#include "../secloc_stream_protocol.h"

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
    hls::stream<secloc_axis32_t>& gate_resp
);

/* Split transaction helpers for single-threaded integration tests. */
void secloc_shell_write_transaction(
    float angleD, float angle_cos, float angle_sin,
    float position, float positionD,
    float target_equilibrium, float target_position,
    float angle, int tick,
    float log_base, int ref_period_ticks,
    float dead_ang, float dead_pos,
    unsigned control_flags,
    hls::stream<secloc_axis32_t>& gate_req);

void secloc_shell_read_transaction(
    hls::stream<secloc_axis32_t>& gate_resp,
    float* Q, unsigned* status,
    unsigned* update_count, unsigned* nn_wait_cycles);

#endif /* SECLOC_SHELL_H */
