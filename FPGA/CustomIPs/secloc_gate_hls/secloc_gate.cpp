/*
 * SecLoc gate block: gate + zero-order hold + downstream watchdog.
 */

#include "secloc_gate.h"

#include "secloc_logic.c"

#ifdef SECLOC_INTEGRATION_TB
#include "../nn_marshal_hls/nn_marshal.h"

static hls::stream<secloc_axis32_t>* g_tb_nn_req  = nullptr;
static hls::stream<secloc_axis32_t>* g_tb_nn_resp = nullptr;

void secloc_gate_set_tb_nn_streams(
    hls::stream<secloc_axis32_t>* nn_req,
    hls::stream<secloc_axis32_t>* nn_resp)
{
    g_tb_nn_req  = nn_req;
    g_tb_nn_resp = nn_resp;
}
#endif

static int gate_should_sample(
    SeclocState*        state,
    const SeclocConfig* config,
    float p, float a, float tp, float te, int tick)
{
#pragma HLS INLINE off
    return secloc_should_sample_tick(state, config, p, a, tp, te, tick);
}

static void read_request_packet(
    hls::stream<secloc_axis32_t>& req_in,
    ap_uint<32> words[SECLOC_REQ_WORDS])
{
READ_REQ:
    for (int i = 0; i < SECLOC_REQ_WORDS; ++i) {
#pragma HLS PIPELINE II=1
        secloc_axis32_t beat = req_in.read();
        words[i] = beat.data;
    }
}

static void write_response_packet(
    hls::stream<secloc_axis32_t>& resp_out,
    float Q,
    unsigned status,
    unsigned updates,
    unsigned wait_cycles)
{
WRITE_RESP:
    for (int i = 0; i < SECLOC_RESP_WORDS; ++i) {
#pragma HLS PIPELINE II=1
        ap_uint<32> data = 0;
        bool last = (i == SECLOC_RESP_WORDS - 1);
        switch (i) {
            case SECLOC_RESP_Q:
                data = secloc_float_to_bits(Q);
                break;
            case SECLOC_RESP_STATUS:
                data = status;
                break;
            case SECLOC_RESP_UPDATE_COUNT:
                data = updates;
                break;
            case SECLOC_RESP_NN_WAIT:
                data = wait_cycles;
                break;
            default:
                break;
        }
        resp_out.write(secloc_make_beat(data, last));
    }
}

static bool forward_to_marshal_and_wait(
    hls::stream<secloc_axis32_t>& marshal_req,
    hls::stream<secloc_axis32_t>& marshal_resp,
    const ap_uint<32> nn_inputs[SECLOC_MARSHAL_INPUTS],
    float* Q_out,
    unsigned* wait_cycles_out)
{
    FORWARD_REQ:
    for (int i = 0; i < SECLOC_MARSHAL_INPUTS; ++i) {
#pragma HLS PIPELINE II=1
        bool last = (i == SECLOC_MARSHAL_INPUTS - 1);
        marshal_req.write(secloc_make_beat(nn_inputs[i], last));
    }

#ifdef SECLOC_INTEGRATION_TB
    if (g_tb_nn_req && g_tb_nn_resp && !g_tb_nn_resp->empty()) {
        nn_marshal_handle_transaction(
            marshal_req, marshal_resp, *g_tb_nn_req, *g_tb_nn_resp);
    }
#endif

    secloc_axis32_t resp;
    bool got_resp = false;
    unsigned wait_cycles = 0u;
MARSHAL_WAIT:
    while (wait_cycles < SECLOC_NN_TIMEOUT_CYCLES) {
#pragma HLS PIPELINE II=1
        if (marshal_resp.read_nb(resp)) {
            got_resp = true;
            break;
        }
        wait_cycles++;
    }

    *wait_cycles_out = wait_cycles;
    if (got_resp) {
        *Q_out = secloc_bits_to_float(resp.data);
        return true;
    }

#ifndef __SYNTHESIS__
    while (!marshal_req.empty()) {
        (void)marshal_req.read();
    }
#endif
    if (marshal_resp.read_nb(resp)) {
        (void)resp;
    }
    return false;
}

void secloc_gate_handle_transaction(
    hls::stream<secloc_axis32_t>& req_in,
    hls::stream<secloc_axis32_t>& resp_out,
    hls::stream<secloc_axis32_t>& marshal_req,
    hls::stream<secloc_axis32_t>& marshal_resp)
{
    static SeclocState gate_state = {0.0001f, 0.0001f, 0.0f, 1u, 0.0f, -1};
    static float       held_Q     = 0.0f;
    static unsigned    updates    = 0u;

    ap_uint<32> req_words[SECLOC_REQ_WORDS];
    read_request_packet(req_in, req_words);

    const unsigned control_flags = (unsigned)req_words[SECLOC_REQ_CONTROL_FLAGS];
    const int        tick          = (int)req_words[SECLOC_REQ_TICK];
    const float      angle         = secloc_bits_to_float(req_words[SECLOC_REQ_ANGLE]);
    const float      position      = secloc_bits_to_float(req_words[SECLOC_REQ_POSITION]);
    const float      te            = secloc_bits_to_float(req_words[SECLOC_REQ_TARGET_EQ]);
    const float      tp            = secloc_bits_to_float(req_words[SECLOC_REQ_TARGET_POS]);

    SeclocConfig gate_cfg;
    gate_cfg.log_base         = secloc_bits_to_float(req_words[SECLOC_REQ_LOG_BASE]);
    gate_cfg.ref_period_ticks = (int)req_words[SECLOC_REQ_REF_PERIOD];
    gate_cfg.ang_dead_band    = secloc_bits_to_float(req_words[SECLOC_REQ_DEAD_ANG]);
    gate_cfg.pos_dead_band    = secloc_bits_to_float(req_words[SECLOC_REQ_DEAD_POS]);
    gate_cfg.time_quantum_s   = 0.0f;

    unsigned status_word = (SECLOC_STREAM_VERSION << 24);
    unsigned wait_cycles = 0u;

    if (control_flags & SECLOC_CTRL_GATE_RESET) {
        secloc_reset(&gate_state);
        held_Q  = 0.0f;
        updates = 0u;
    } else {
        const bool force = (control_flags & SECLOC_CTRL_FORCE_COMPUTE) != 0u;

        int gate_eval = 0;
        int fire      = 0;
        if (force) {
            fire = 1;
        } else {
            gate_eval = secloc_gate_evaluated_tick(&gate_state, &gate_cfg, tick);
            fire = gate_should_sample(&gate_state, &gate_cfg,
                                      position, angle, tp, te, tick);
        }

        if (fire) {
            ap_uint<32> marshal_inputs[SECLOC_MARSHAL_INPUTS];
            marshal_inputs[0] = req_words[SECLOC_REQ_ANGLED];
            marshal_inputs[1] = req_words[SECLOC_REQ_ANGLE_COS];
            marshal_inputs[2] = req_words[SECLOC_REQ_ANGLE_SIN];
            marshal_inputs[3] = req_words[SECLOC_REQ_POSITION];
            marshal_inputs[4] = req_words[SECLOC_REQ_POSITIOND];
            marshal_inputs[5] = req_words[SECLOC_REQ_TARGET_EQ];
            marshal_inputs[6] = req_words[SECLOC_REQ_TARGET_POS];

            float new_Q = held_Q;
            if (forward_to_marshal_and_wait(
                    marshal_req, marshal_resp, marshal_inputs,
                    &new_Q, &wait_cycles)) {
                held_Q = new_Q;
                updates++;
                status_word |= SECLOC_STATUS_FIRED;
            } else {
                status_word |= SECLOC_STATUS_NN_TIMEOUT | SECLOC_STATUS_SKIPPED;
            }
        } else {
            status_word |= SECLOC_STATUS_SKIPPED;
        }

        if (!force && gate_eval) {
            status_word |= SECLOC_STATUS_GATE_EVALUATED;
        }
    }

    write_response_packet(resp_out, held_Q, status_word, updates, wait_cycles);
}

void secloc_gate(
    hls::stream<secloc_axis32_t>& req_in,
    hls::stream<secloc_axis32_t>& resp_out,
    hls::stream<secloc_axis32_t>& marshal_req,
    hls::stream<secloc_axis32_t>& marshal_resp)
{
#pragma HLS INTERFACE axis port=req_in
#pragma HLS INTERFACE axis port=resp_out
#pragma HLS INTERFACE axis port=marshal_req
#pragma HLS INTERFACE axis port=marshal_resp
#pragma HLS INTERFACE ap_ctrl_none port=return

GATE_LOOP:
    while (true) {
#pragma HLS PIPELINE off
        secloc_gate_handle_transaction(req_in, resp_out, marshal_req, marshal_resp);
    }
}
