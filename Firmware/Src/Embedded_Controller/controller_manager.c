#include "controller_manager.h"
#include "hardware_bridge.h"  /* Message_GetFromPC / Message_SendToPC */
#include <string.h>

static const ControllerOps* g_active  = 0;
/* controller we will switch to after PC re-handshakes */
static const ControllerOps* g_pending = 0;
/* one-shot cookie flag + generation counter (just for uniqueness) */
static uint8_t g_cookie_pending = 0;
static uint8_t g_spec_gen       = 1;
/* handshake completion flag */
static int g_handshake_complete = 0;

void CR_SetActive(const ControllerOps* ops) { g_active = ops; }
const ControllerOps* CR_GetActive(void)     { return g_active; }

uint8_t CR_ActiveNumInputs(void)  { return g_active ? g_active->spec()->n_inputs  : 0; }
uint8_t CR_ActiveNumOutputs(void) { return g_active ? g_active->spec()->n_outputs : 0; }

/* Check if handshake protocol has been completed */
int CR_IsHandshakeComplete(void) { return g_handshake_complete; }

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
    unsigned char cookie[4] = { SERIAL_SOF, MSG_TYPE_SPEC_COOKIE, 4, 0 };
    cookie[3] = crc8_calc(cookie, 3);
    Message_SendToPC_blocking(cookie, 4);
    return 1;
}

/* Serve a single framed GET_SPEC: request [SOF, CMD, 4, CRC] -> reply 4B header + names. */
// if a switch is pending, finalize it *here* so PC gets the new spec.
void CR_HandshakeOnce(void)
{
    static unsigned char cmd[4];
    static int got = 0;
    static int timeout_counter = 0;
    
    /* Try to get 4 bytes, but don't block if not available */
    while (got < 4) {
        int n = Message_GetFromPC(&cmd[got]);
        if (n > 0) {
            got += n;
            timeout_counter = 0;  // Reset timeout on data received
            // xil_printf("DEBUG: Received %d bytes, total: %d/4\r\n", n, got);
            // Print the actual bytes received
            // for (int i = 0; i < n; i++) {
            //     xil_printf("DEBUG: Byte %d: %02X\r\n", got - n + i, cmd[got - n + i]);
            // }
        } else {
            /* No data available, increment timeout counter */
            timeout_counter++;
            if (timeout_counter > 10000) {  // Reset partial command after timeout
                got = 0;
                timeout_counter = 0;
                // xil_printf("DEBUG: Timeout, resetting command buffer\r\n");
            }
            return;
        }
    }

    // xil_printf("DEBUG: Got 4 bytes: %02X %02X %02X %02X\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);

    if (!g_active || !g_active->spec) {
        // xil_printf("DEBUG: No active controller or spec\r\n");
        got = 0;  // Reset for next command
        return;
    }

    if (cmd[0] == SERIAL_SOF && cmd[1] == CMD_GET_SPEC && cmd[2] == 4 && cmd[3] == crc8_calc(cmd, 3)) {
        // xil_printf("DEBUG: Valid GET_SPEC received, processing...\r\n");

        /* If this is a new handshake request, reset the handshake state */
        if (g_handshake_complete) {
            // xil_printf("DEBUG: New handshake detected, resetting state\r\n");
            g_handshake_complete = 0;
        }

        /* finalize any pending controller switch right before replying */
        if (g_pending) {
            // xil_printf("DEBUG: Switching controller\r\n");
            if (g_active->release) g_active->release();
            g_active = g_pending;
            g_pending = 0;
            if (g_active->init) g_active->init();
            g_cookie_pending = 0; /* cookie served its purpose */
        }

        const ControllerSpec* N = g_active->spec();
        unsigned char hdr[4] = { N->version, N->n_inputs, N->n_outputs, NAME_TOKEN_LEN };
        // xil_printf("DEBUG: Sending header: version=%d, n_inputs=%d, n_outputs=%d, token_len=%d\r\n", 
        //            hdr[0], hdr[1], hdr[2], hdr[3]);
        
        Message_SendToPC_blocking(hdr, 4);
        // xil_printf("DEBUG: Header sent, sending %d input names...\r\n", N->n_inputs);
        
        for (uint8_t i = 0; i < N->n_inputs; ++i) {
            // xil_printf("DEBUG: Sending name %d: %s\r\n", i, N->names[i]);
            write_padded_token(N->names[i]);
        }
        // xil_printf("DEBUG: Handshake complete\r\n");
        
        /* Mark handshake as complete - allow re-handshaking */
        g_handshake_complete = 1;
        got = 0;  // Reset for next command
    } else if (cmd[0] == SERIAL_SOF && cmd[1] == CMD_PING && cmd[2] == 4 && cmd[3] == crc8_calc(cmd, 3)) {
        // xil_printf("DEBUG: PING received, echoing back\r\n");
        // xil_printf("DEBUG: Echoing: %02X %02X %02X %02X\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);
        Message_SendToPC_blocking(cmd, 4);  // Echo the ping back
        // xil_printf("DEBUG: PING echo sent\r\n");
        got = 0;  // Reset for next command
    } else {
        // xil_printf("DEBUG: Invalid command - SOF=%02X (expected %02X), CMD=%02X (expected %02X or %02X), LEN=%d (expected 4), CRC=%02X (expected %02X)\r\n",
        //            cmd[0], SERIAL_SOF, cmd[1], CMD_GET_SPEC, CMD_PING, cmd[2], cmd[3], crc8_calc(cmd, 3));
        got = 0;  // Reset for next command
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

/* reset handshake state to allow new handshakes */
void CR_ResetHandshakeState(void)
{
    g_handshake_complete = 0;
    g_cookie_pending = 0;
    g_pending = 0;
}

/* ===== NEW UNIFIED MESSAGE PROCESSING ===== */

/* Process a single message from PC. Returns 1 if message processed, 0 if no data available */
int CR_ProcessMessage(void)
{
    static unsigned char msg_buffer[4 + MAX_INPUTS * 4]; /* SOF + type + length + max data + CRC */
    static int got = 0;
    static int timeout_counter = 0;
    
    /* Try to get at least 4 bytes (SOF + type + length + CRC) */
    while (got < 4) {
        int n = Message_GetFromPC(&msg_buffer[got]);
        if (n > 0) {
            got += n;
            timeout_counter = 0;
        } else {
            timeout_counter++;
            if (timeout_counter > 10000) {
                got = 0;
                timeout_counter = 0;
            }
            return 0; /* No data available */
        }
    }
    
    /* Check if this is a valid message start */
    if (msg_buffer[0] != SERIAL_SOF) {
        /* Invalid start, shift buffer and try again */
        for (int i = 0; i < got - 1; i++) {
            msg_buffer[i] = msg_buffer[i + 1];
        }
        got--;
        return 1; /* Try again */
    }
    
    uint8_t msg_type = msg_buffer[1];
    uint8_t msg_length = msg_buffer[2];
    
    /* Validate message length */
    if (msg_length < 4 || msg_length > sizeof(msg_buffer)) {
        /* Invalid length, reset */
        got = 0;
        return 1;
    }
    
    /* Check if we need more data */
    if (got < msg_length) {
        int n = Message_GetFromPC(&msg_buffer[got]);
        if (n > 0) {
            got += n;
            timeout_counter = 0;
        } else {
            timeout_counter++;
            if (timeout_counter > 10000) {
                got = 0;
                timeout_counter = 0;
            }
        }
        return 0; /* Still waiting for more data */
    }
    
    /* We have a complete message, verify CRC */
    if (msg_buffer[msg_length - 1] != crc8_calc(msg_buffer, msg_length - 1)) {
        /* CRC failed, reset */
        got = 0;
        return 1;
    }
    
    /* Process the message based on type */
    switch (msg_type) {
        case MSG_TYPE_STATE:
            if (msg_length >= 4) {
                CR_ProcessStateData(&msg_buffer[3], msg_length - 4); /* Skip SOF, type, length, CRC */
            }
            break;
            
        case MSG_TYPE_GET_SPEC:
            CR_ProcessSpecRequest();
            break;
            
        case MSG_TYPE_PING:
            CR_ProcessPing();
            break;
            
        default:
            /* Unknown message type, ignore */
            break;
    }
    
    /* Reset for next message */
    got = 0;
    return 1;
}

/* Process state data for controller */
void CR_ProcessStateData(const unsigned char* data, unsigned int length)
{
    if (!g_active || !g_active->spec) return;
    
    const uint8_t nin = g_active->spec()->n_inputs;
    const unsigned int expected_bytes = nin * 4;
    
    if (length != expected_bytes) {
        /* Wrong data length, ignore */
        return;
    }
    
    /* Convert bytes to floats and evaluate controller */
    static unsigned char tx_buffer[MAX_OUTPUTS * 4];
    CR_EvaluateBytes(data, tx_buffer);
    
    /* Send controller output */
    const uint8_t nout = g_active->spec()->n_outputs;
    Message_SendToPC(tx_buffer, nout * 4);
}

/* Process spec request */
void CR_ProcessSpecRequest(void)
{
    if (!g_active || !g_active->spec) return;
    
    /* If this is a new handshake request, reset the handshake state */
    if (g_handshake_complete) {
        g_handshake_complete = 0;
    }
    
    /* Finalize any pending controller switch right before replying */
    if (g_pending) {
        if (g_active->release) g_active->release();
        g_active = g_pending;
        g_pending = 0;
        if (g_active->init) g_active->init();
        g_cookie_pending = 0;
    }
    
    const ControllerSpec* N = g_active->spec();
    unsigned char hdr[4] = { N->version, N->n_inputs, N->n_outputs, NAME_TOKEN_LEN };
    
    Message_SendToPC_blocking(hdr, 4);
    
    for (uint8_t i = 0; i < N->n_inputs; ++i) {
        write_padded_token(N->names[i]);
    }
    
    /* Mark handshake as complete */
    g_handshake_complete = 1;
}

/* Process ping request */
void CR_ProcessPing(void)
{
    /* Echo back the ping */
    unsigned char ping_response[4] = { SERIAL_SOF, MSG_TYPE_PING, 4, 0 };
    ping_response[3] = crc8_calc(ping_response, 3);
    Message_SendToPC_blocking(ping_response, 4);
}