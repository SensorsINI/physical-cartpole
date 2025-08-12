#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller_api.h"

/* Upper bounds to allocate once (adjust conservatively if needed). */
#define MAX_INPUTS   64
#define MAX_OUTPUTS  16

/* Control-plane UART protocol shared with the PC. */
#define SERIAL_SOF     0xAA
#define CMD_GET_SPEC  0xC6
#define NAME_TOKEN_LEN 16   /* fixed-size ASCII tokens, NUL padded */

// out-of-band cookie announcing “spec will change; re-handshake now”.
#define CMD_SPEC_COOKIE  0xC7

void                 CR_SetActive(const ControllerOps* ops);
const ControllerOps* CR_GetActive(void);

uint8_t CR_ActiveNumInputs(void);
uint8_t CR_ActiveNumOutputs(void);

/* One-time (or on-demand) handshake: serves GET_SPEC and replies with header+tokens. */
void CR_HandshakeOnce(void);

/* Data-plane bridge: converts UART bytes <-> float arrays and dispatches. */
void CR_EvaluateBytes(const unsigned char* in_bytes, unsigned char* out_bytes);

/* request switching to another controller at the next safe boundary. */
void CR_RequestSwitch(const ControllerOps* new_ops);

/* emit a one-shot spec cookie if a switch was requested; returns 1 if sent. */
int  CR_SendSpecCookieIfPending(void);

#endif /* CONTROLLER_MANAGER_H */
