/*
 * Shell-only testbench: verify register-to-packet marshalling with a mock gate.
 */

#include <cstdio>

#include "secloc_shell.h"

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

static void mock_gate_once(
    hls::stream<secloc_axis32_t>& req_in,
    hls::stream<secloc_axis32_t>& resp_out)
{
    ap_uint<32> words[SECLOC_REQ_WORDS];
    for (int i = 0; i < SECLOC_REQ_WORDS; ++i) {
        words[i] = req_in.read().data;
    }

    const unsigned flags = (unsigned)words[SECLOC_REQ_CONTROL_FLAGS];
    float Q = 3.14f;
    unsigned status = (SECLOC_STREAM_VERSION << 24) | SECLOC_STATUS_FIRED;
    unsigned updates = 7u;
    unsigned waited = 42u;

    if (flags & SECLOC_CTRL_GATE_RESET) {
        Q = 0.0f;
        status = (SECLOC_STREAM_VERSION << 24);
        updates = 0u;
        waited = 0u;
    }

    ap_uint<32> resp_words[SECLOC_RESP_WORDS] = {
        secloc_float_to_bits(Q),
        status,
        updates,
        waited
    };
    for (int i = 0; i < SECLOC_RESP_WORDS; ++i) {
        resp_out.write(secloc_make_beat(resp_words[i], i == SECLOC_RESP_WORDS - 1));
    }
}

int main()
{
    hls::stream<secloc_axis32_t> gate_req;
    hls::stream<secloc_axis32_t> gate_resp;

    secloc_shell_write_transaction(
        0.1f, 0.99f, 0.14f, 0.05f, 0.01f, 1.0f, 0.0f, 0.14f, 3,
        1.05f, 4, 0.001f, 0.001f, 0u, gate_req);
    mock_gate_once(gate_req, gate_resp);

    float Q = 0.0f;
    unsigned status = 0u, updates = 0u, waited = 0u;
    secloc_shell_read_transaction(gate_resp, &Q, &status, &updates, &waited);

    CHECK(Q == 3.14f, "Q mismatch: %f", Q);
    CHECK((status & SECLOC_STATUS_FIRED) != 0u, "status fired bit missing");
    CHECK(updates == 7u, "updates mismatch: %u", updates);
    CHECK(waited == 42u, "waited mismatch: %u", waited);

    if (failures == 0) {
        std::printf("secloc_shell_tb: ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("secloc_shell_tb: %d FAILURES\n", failures);
    return 1;
}
