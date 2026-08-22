#ifndef SECLOC_GATE_H
#define SECLOC_GATE_H

/*
 * SecLoc gate block: AXI-Stream in/out on both sides, ap_ctrl_none.
 *
 * Upstream (shell): 14-word request / 4-word response packets per
 * secloc_stream_protocol.h.
 * Downstream (nn_marshal): 7 raw float32 beats on fire, 1 float32 Q back.
 *
 * Contains the SecLoc gate (firmware secloc_logic.c), zero-order hold,
 * update_count, and the downstream watchdog.
 */

#include <hls_stream.h>
#include "../secloc_stream_protocol.h"

void secloc_gate(
    hls::stream<secloc_axis32_t>& req_in,
    hls::stream<secloc_axis32_t>& resp_out,
    hls::stream<secloc_axis32_t>& marshal_req,
    hls::stream<secloc_axis32_t>& marshal_resp
);

/* One upstream transaction (14-word request in, 4-word response out). */
void secloc_gate_handle_transaction(
    hls::stream<secloc_axis32_t>& req_in,
    hls::stream<secloc_axis32_t>& resp_out,
    hls::stream<secloc_axis32_t>& marshal_req,
    hls::stream<secloc_axis32_t>& marshal_resp
);

#ifdef SECLOC_INTEGRATION_TB
void secloc_gate_set_tb_nn_streams(
    hls::stream<secloc_axis32_t>* nn_req,
    hls::stream<secloc_axis32_t>* nn_resp
);
#endif

#endif /* SECLOC_GATE_H */
