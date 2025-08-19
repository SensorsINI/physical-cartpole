#include "controller_manager.h"
#include "hardware_bridge.h"  /* Message_GetFromPC / Message_SendToPC */
#include <string.h>

static const ControllerOps* g_active  = 0;
/* controller we will switch to after PC re-handshakes */
static const ControllerOps* g_pending = 0;
/* one-shot cookie flag + generation counter (just for uniqueness) */
static uint8_t g_cookie_pending = 0;
static uint8_t g_spec_gen       = 1;

void CR_SetActive(const ControllerOps* ops) { g_active = ops; }
const ControllerOps* CR_GetActive(void)     { return g_active; }

uint8_t CR_ActiveNumInputs(void)  { return g_active ? g_active->spec()->n_inputs  : 0; }
uint8_t CR_ActiveNumOutputs(void) { return g_active ? g_active->spec()->n_outputs : 0; }

/* CRC8 (poly 0x8C), LSB-first, bytewise—exactly matches PC implementation. */
static uint8_t crc8_calc(const uint8_t *msg, size_t len) {
    uint8_t crc8 = 0x00;
    for (size_t i = 0; i < len; ++i) {
        uint8_t val = msg[i];
        for (int b = 0; b < 8; ++b) {
            uint8_t s = (uint8_t)((crc8 ^ val) & 0x01);
            crc8 >>= 1;
            if (s) crc8 ^= 0x8C;
            val >>= 1;
        }
    }
    return crc8;
}

/* Emit one NAME_TOKEN_LEN-byte ASCII token padded with NULs. */
static void write_padded_token(const char* s) {
    unsigned char tok[NAME_TOKEN_LEN];
    size_t i = 0;
    for (; i < NAME_TOKEN_LEN && s[i] != '\0'; ++i) tok[i] = (unsigned char)s[i];
    for (; i < NAME_TOKEN_LEN; ++i) tok[i] = 0;
    Message_SendToPC_blocking(tok, NAME_TOKEN_LEN);
}

/* ask manager to switch controllers; cookie will announce change to PC. */
void CR_RequestSwitch(const ControllerOps* new_ops)
{
    if (!new_ops) return;
    if (new_ops == g_active || new_ops == g_pending) return;
    g_pending        = new_ops;
    g_cookie_pending = 1;
    ++g_spec_gen; /* wrap is fine */
}

/* send the 4B cookie BEFORE outputs to prompt PC to re-handshake. */
int CR_SendSpecCookieIfPending(void)
{
    if (!g_cookie_pending) return 0;
    unsigned char cookie[4] = { SERIAL_SOF, CMD_SPEC_COOKIE, g_spec_gen, 0 };
    cookie[3] = crc8_calc(cookie, 3);
    Message_SendToPC_blocking(cookie, 4);
    return 1;
}

/* Serve a single framed GET_SPEC: request [SOF, CMD, 4, CRC] -> reply 4B header + names. */
// if a switch is pending, finalize it *here* so PC gets the new spec.
void CR_HandshakeOnce(void)
{
    unsigned char cmd[4];
    unsigned int got = 0;

    while (got < sizeof(cmd)) {
        int n = Message_GetFromPC(&cmd[got]);   /* blocks until header arrives */
        if (n > 0) got += (unsigned int)n;
    }
    if (!g_active || !g_active->spec) return;

    if (cmd[0] == SERIAL_SOF && cmd[1] == CMD_GET_SPEC && cmd[2] == 4 && cmd[3] == crc8_calc(cmd, 3)) {

        /* finalize any pending controller switch right before replying */
        if (g_pending) {
            if (g_active->release) g_active->release();
            g_active = g_pending;
            g_pending = 0;
            if (g_active->init) g_active->init();
            g_cookie_pending = 0; /* cookie served its purpose */
        }

        const ControllerSpec* N = g_active->spec();
        unsigned char hdr[4] = { N->version, N->n_inputs, N->n_outputs, NAME_TOKEN_LEN };
        Message_SendToPC_blocking(hdr, 4);
        for (uint8_t i = 0; i < N->n_inputs; ++i) write_padded_token(N->names[i]);
    }
    /* Malformed? Ignore; PC will timeout and retry. */
}

/* Convert little-endian float32 bytes <-> float arrays without aliasing/alignment UB. */
void CR_EvaluateBytes(const unsigned char* in_b, unsigned char* out_b)
{
    const ControllerSpec* N = g_active->spec();
    float in_f [MAX_INPUTS];
    float out_f[MAX_OUTPUTS];

    const uint8_t nin  = N->n_inputs;
    const uint8_t nout = N->n_outputs;

    for (uint8_t i = 0; i < nin;  ++i) memcpy(&in_f[i],        &in_b[i*4], 4);
    if (g_active->evaluate) g_active->evaluate(in_f, out_f);
    for (uint8_t i = 0; i < nout; ++i) memcpy(&out_b[i*4],     &out_f[i],  4);
}
