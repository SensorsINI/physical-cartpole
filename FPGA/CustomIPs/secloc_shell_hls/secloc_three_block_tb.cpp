/*
 * Integration testbench: shell + gate + marshal chained synchronously through
 * AXI-Stream, with a mock network on the far side.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

#include "secloc_shell.h"
#include "../secloc_gate_hls/secloc_gate.h"
#include "secloc_logic.h"
#include "nn_marshal_config.h"
#include "../nn_fixed_format.h"

static int failures = 0;

#define CHECK(cond, ...)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAIL %s:%d: ", __FILE__, __LINE__);                 \
            std::printf(__VA_ARGS__);                                        \
            std::printf("\n");                                               \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static unsigned ref_normalize_quantize(float x, int idx)
{
    const float y = NN_NORM_A[idx] * x + NN_NORM_B[idx];
    return (unsigned)nn_fixed_quantize_input(y);
}

static float ref_dequantize_denorm(unsigned code12)
{
    float val = nn_fixed_dequantize_output((ap_uint<32>)code12);
    return NN_DENORM_A[0] * val + NN_DENORM_B[0];
}

struct TbStreams {
    hls::stream<secloc_axis32_t> shell_to_gate_req;
    hls::stream<secloc_axis32_t> gate_to_shell_resp;
    hls::stream<secloc_axis32_t> gate_to_marshal_req;
    hls::stream<secloc_axis32_t> marshal_to_gate_resp;
    hls::stream<secloc_axis32_t> marshal_to_nn_req;
    hls::stream<secloc_axis32_t> nn_to_marshal_resp;
};

struct TbState {
    SeclocState  ref_gate;
    SeclocConfig ref_cfg;
    float        expected_Q;
    unsigned     expected_updates;
    TbStreams    streams;

    TbState()
    {
        secloc_reset(&ref_gate);
        ref_cfg.log_base         = 1.05f;
        ref_cfg.ref_period_ticks = 0;
        ref_cfg.ang_dead_band    = 0.001f;
        ref_cfg.pos_dead_band    = 0.001f;
        ref_cfg.time_quantum_s   = 0.0f;
        expected_Q       = 0.0f;
        expected_updates = 0u;
        secloc_gate_set_tb_nn_streams(
            &streams.marshal_to_nn_req, &streams.nn_to_marshal_resp);
    }
};

static void step(TbState& tb,
                 float angleD, float angle_cos, float angle_sin,
                 float position, float positionD,
                 float te, float tp, float angle, int tick,
                 unsigned control_flags, unsigned mock_code,
                 bool starve_nn = false);

static void reset_dut(TbState& tb)
{
    step(tb, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1,
         SECLOC_CTRL_GATE_RESET, 0u, false);
}

static void step(TbState& tb,
                 float angleD, float angle_cos, float angle_sin,
                 float position, float positionD,
                 float te, float tp, float angle, int tick,
                 unsigned control_flags, unsigned mock_code,
                 bool starve_nn)
{
    const bool is_reset = (control_flags & SECLOC_CTRL_GATE_RESET) != 0u;
    const bool is_force = (control_flags & SECLOC_CTRL_FORCE_COMPUTE) != 0u;

    int ref_eval = 0;
    int ref_fire = 0;
    if (!is_reset) {
        if (is_force) {
            ref_fire = 1;
        } else {
            ref_eval = secloc_gate_evaluated_tick(&tb.ref_gate, &tb.ref_cfg, tick);
            ref_fire = secloc_should_sample_tick(&tb.ref_gate, &tb.ref_cfg,
                                                 position, angle, tp, te, tick);
        }
    }

    if (ref_fire && !starve_nn) {
        secloc_axis32_t resp;
        resp.data = mock_code & 0xFFFu;
        resp.keep = 0xF;
        resp.strb = 0xF;
        resp.last = 1;
        tb.streams.nn_to_marshal_resp.write(resp);
    }

    secloc_shell_write_transaction(
        angleD, angle_cos, angle_sin, position, positionD,
        te, tp, angle, tick,
        tb.ref_cfg.log_base, tb.ref_cfg.ref_period_ticks,
        tb.ref_cfg.ang_dead_band, tb.ref_cfg.pos_dead_band,
        control_flags, tb.streams.shell_to_gate_req);

    secloc_gate_handle_transaction(
        tb.streams.shell_to_gate_req, tb.streams.gate_to_shell_resp,
        tb.streams.gate_to_marshal_req, tb.streams.marshal_to_gate_resp);

    float    Q       = -1e9f;
    unsigned status  = 0u;
    unsigned updates = 0u;
    unsigned waited  = 0u;

    secloc_shell_read_transaction(
        tb.streams.gate_to_shell_resp, &Q, &status, &updates, &waited);

    if (is_reset) {
        secloc_reset(&tb.ref_gate);
        tb.expected_Q       = 0.0f;
        tb.expected_updates = 0u;
    } else if (ref_fire && !starve_nn) {
        tb.expected_Q = ref_dequantize_denorm(mock_code);
        tb.expected_updates++;
    }

    CHECK((status >> 24) == SECLOC_STREAM_VERSION,
          "version field: got %u", status >> 24);

    if (is_reset) {
        CHECK(tb.streams.marshal_to_nn_req.empty(), "reset must not touch the network");
        CHECK(updates == 0u, "reset must clear update_count (got %u)", updates);
        CHECK(Q == 0.0f, "reset must clear held Q (got %f)", Q);
        return;
    }

    const bool dut_fired   = (status & SECLOC_STATUS_FIRED) != 0u;
    const bool dut_skipped = (status & SECLOC_STATUS_SKIPPED) != 0u;
    const bool dut_eval    = (status & SECLOC_STATUS_GATE_EVALUATED) != 0u;
    const bool dut_timeout = (status & SECLOC_STATUS_NN_TIMEOUT) != 0u;

    if (ref_fire && !starve_nn) {
        CHECK(dut_fired && !dut_skipped && !dut_timeout,
              "expected fire, status=0x%x", status);
        const float x[SECLOC_NN_INPUTS] = {
            angleD, angle_cos, angle_sin, position, positionD, te, tp
        };
        for (int i = 0; i < SECLOC_NN_INPUTS; ++i) {
            CHECK(!tb.streams.marshal_to_nn_req.empty(), "request beat %d missing", i);
            if (tb.streams.marshal_to_nn_req.empty()) break;
            secloc_axis32_t beat = tb.streams.marshal_to_nn_req.read();
            unsigned expect = ref_normalize_quantize(x[i], i);
            CHECK((unsigned)beat.data == expect,
                  "beat %d: code 0x%x != expected 0x%x",
                  i, (unsigned)beat.data, expect);
            CHECK((unsigned)beat.last == (i == SECLOC_NN_INPUTS - 1 ? 1u : 0u),
                  "beat %d: TLAST wrong", i);
        }
    } else if (ref_fire && starve_nn) {
        CHECK(dut_timeout && dut_skipped && !dut_fired,
              "expected timeout, status=0x%x", status);
        CHECK(waited >= SECLOC_NN_TIMEOUT_CYCLES,
              "timeout must exhaust the wait budget (%u)", waited);
        while (!tb.streams.marshal_to_nn_req.empty()) {
            (void)tb.streams.marshal_to_nn_req.read();
        }
    } else {
        CHECK(!dut_fired && dut_skipped, "expected skip, status=0x%x", status);
        CHECK(tb.streams.marshal_to_nn_req.empty(), "skip must not touch the network");
    }

    if (!is_force) {
        CHECK(dut_eval == (ref_eval != 0),
              "gate_evaluated: dut=%d ref=%d", (int)dut_eval, ref_eval);
    } else {
        CHECK(!dut_eval, "force mode must not report gate_evaluated");
    }

    CHECK(Q == tb.expected_Q, "Q: dut=%.9g expected=%.9g", Q, tb.expected_Q);
    CHECK(updates == tb.expected_updates,
          "update_count: dut=%u expected=%u", updates, tb.expected_updates);
}

int main()
{
    TbState tb;

    std::mt19937 rng(20260709u);
    std::uniform_real_distribution<float> uni(-1.0f, 1.0f);
    std::uniform_int_distribution<unsigned> code_dist(0u, 0xFFFu);

    {
        reset_dut(tb);
        step(tb, 0.1f, 0.99f, 0.14f, 0.00f, 0.0f, 1.0f, 0.0f, 0.14f, -1, 0, 0x123u);
        step(tb, 0.1f, 0.99f, 0.14f, 0.00f, 0.0f, 1.0f, 0.0f, 0.14f, -1, 0, 0x456u);
        step(tb, 0.1f, 0.99f, 0.14f, 0.05f, 0.0f, 1.0f, 0.0f, 0.14f, -1, 0, 0xFFFu);
        step(tb, 0.1f, 0.99f, 0.14f, 0.0505f, 0.0f, 1.0f, 0.0f, 0.14f, -1, 0, 0x0u);
    }

    {
        reset_dut(tb);
        const float pi = 3.14159265358979323846f;
        step(tb, 0.0f, -1.0f, 0.05f, 0.0f, 0.0f, -1.0f, 0.0f,  pi - 0.05f, -1, 0, 0x200u);
        step(tb, 0.0f, -1.0f, 0.05f, 0.0f, 0.0f, -1.0f, 0.0f,  pi - 0.051f, -1, 0, 0x201u);
        step(tb, 0.0f, -1.0f, 0.06f, 0.0f, 0.0f, -1.0f, 0.0f,  pi - 0.06f, -1, 0, 0x202u);
        step(tb, 0.0f, -1.0f, 0.06f, 0.0f, 0.0f, -1.0f, 0.0f, -(pi - 0.06f), -1, 0, 0x203u);
    }

    {
        reset_dut(tb);
        tb.ref_cfg.ref_period_ticks = 4;
        step(tb, 0.0f, 1.0f, 0.0f, 0.00f, 0.0f, 1.0f, 0.0f, 0.30f, 0, 0, 0x100u);
        step(tb, 0.0f, 1.0f, 0.0f, 0.00f, 0.0f, 1.0f, 0.0f, 0.60f, 1, 0, 0x101u);
        step(tb, 0.0f, 1.0f, 0.0f, 0.00f, 0.0f, 1.0f, 0.0f, 0.60f, 3, 0, 0x102u);
        step(tb, 0.0f, 1.0f, 0.0f, 0.00f, 0.0f, 1.0f, 0.0f, 0.60f, 4, 0, 0x103u);
    }

    {
        reset_dut(tb);
        step(tb, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.20f, -1, 0, 0x111u);
        step(tb, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.20f, -1,
             SECLOC_CTRL_FORCE_COMPUTE, 0x222u);
        step(tb, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.20f, -1, 0, 0x333u);
        step(tb, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.20f, -1,
             SECLOC_CTRL_GATE_RESET, 0u);
        step(tb, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.20f, -1, 0, 0x444u);
    }

    {
        reset_dut(tb);
        step(tb, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.25f, -1, 0, 0x0u, true);
        step(tb, 0.0f, 1.0f, 0.0f, 0.1f, 0.0f, 1.0f, 0.0f, 0.50f, -1, 0, 0x055u);
    }

    {
        reset_dut(tb);
        tb.ref_cfg.ref_period_ticks = 4;
        const float pi = 3.14159265358979323846f;
        float position = 0.0f;
        for (int i = 0; i < 20000; ++i) {
            int regime = (i / 2000) % 4;
            float te = (regime < 2) ? 1.0f : -1.0f;
            float angle;
            if (regime == 0)      angle = 0.02f * uni(rng);
            else if (regime == 2) angle = (uni(rng) > 0 ? 1.0f : -1.0f) * (pi - 0.05f * std::fabs(uni(rng)));
            else                  angle = pi * std::sin(0.01f * (float)i) + 0.1f * uni(rng);
            if (angle >  pi) angle =  pi;
            if (angle < -pi) angle = -pi;
            position += 0.002f * uni(rng);
            if (position >  0.198f) position =  0.198f;
            if (position < -0.198f) position = -0.198f;
            float tp = (i < 10000) ? 0.0f : 0.05f;

            step(tb, 0.5f * uni(rng), std::cos(angle), std::sin(angle),
                 position, 0.3f * uni(rng), te, tp, angle, i,
                 0u, code_dist(rng));
            if (failures > 20) break;
        }
    }

    {
        reset_dut(tb);
        for (int i = 0; i < 200; ++i) {
            step(tb, uni(rng), uni(rng), uni(rng), 0.19f * uni(rng),
                 uni(rng), 1.0f, 0.05f * uni(rng), uni(rng), -1,
                 SECLOC_CTRL_FORCE_COMPUTE, code_dist(rng));
            if (failures > 20) break;
        }
    }

    if (failures == 0) {
        std::printf("secloc_three_block_tb: ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("secloc_three_block_tb: %d FAILURES\n", failures);
    return 1;
}
