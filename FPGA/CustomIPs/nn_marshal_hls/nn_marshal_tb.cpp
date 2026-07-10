/*
 * Marshal-only testbench: normalize/denormalize codes vs the PS float path.
 */

#include <cstdio>

#include "nn_marshal.h"
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

int main()
{
    hls::stream<secloc_axis32_t> gate_req;
    hls::stream<secloc_axis32_t> gate_resp;
    hls::stream<secloc_axis32_t> nn_req;
    hls::stream<secloc_axis32_t> nn_resp;

    const float inputs[NN_IO_INPUT_COUNT] = {
        0.1f, 0.99f, 0.14f, 0.05f, 0.01f, 1.0f, 0.0f
    };
    const unsigned mock_code = 0x2A3u;

    for (int i = 0; i < NN_IO_INPUT_COUNT; ++i) {
        gate_req.write(secloc_make_beat(secloc_float_to_bits(inputs[i]),
                                        i == NN_IO_INPUT_COUNT - 1));
    }

    secloc_axis32_t nn_resp_beat;
    nn_resp_beat.data = mock_code;
    nn_resp_beat.keep = 0xF;
    nn_resp_beat.strb = 0xF;
    nn_resp_beat.last = 1;
    nn_resp.write(nn_resp_beat);

    nn_marshal_handle_transaction(gate_req, gate_resp, nn_req, nn_resp);

    secloc_axis32_t gate_answer = gate_resp.read();
    float Q = secloc_bits_to_float(gate_answer.data);
    CHECK(Q == ref_dequantize_denorm(mock_code), "Q mismatch: %f", Q);

    for (int i = 0; i < NN_IO_INPUT_COUNT; ++i) {
        CHECK(!nn_req.empty(), "missing nn beat %d", i);
        secloc_axis32_t beat = nn_req.read();
        unsigned expect = ref_normalize_quantize(inputs[i], i);
        CHECK((unsigned)beat.data == expect,
              "beat %d: code 0x%x != 0x%x", i, (unsigned)beat.data, expect);
    }

    if (failures == 0) {
        std::printf("nn_marshal_tb: ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("nn_marshal_tb: %d FAILURES\n", failures);
    return 1;
}
