#ifndef NN_MARSHAL_H
#define NN_MARSHAL_H

/*
 * Per-network marshal block: AXI-Stream in/out on both sides, ap_ctrl_none.
 *
 * Upstream (secloc_gate): 7 raw float32 beats, 1 float32 Q back.
 * Downstream (controller_axis): 7 x 12-bit codes in 32-bit beats, 1 code back.
 *
 * Per-network constants from nn_marshal_config.h (auto-generated on network
 * conversion). Re-synthesize this IP on a network swap.
 */

#include <hls_stream.h>
#include "../secloc_stream_protocol.h"

void nn_marshal(
    hls::stream<secloc_axis32_t>& gate_req,
    hls::stream<secloc_axis32_t>& gate_resp,
    hls::stream<secloc_axis32_t>& nn_req,
    hls::stream<secloc_axis32_t>& nn_resp
);

/* One downstream transaction (7 floats in, 1 float Q out). */
void nn_marshal_handle_transaction(
    hls::stream<secloc_axis32_t>& gate_req,
    hls::stream<secloc_axis32_t>& gate_resp,
    hls::stream<secloc_axis32_t>& nn_req,
    hls::stream<secloc_axis32_t>& nn_resp
);

#endif /* NN_MARSHAL_H */
