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

void                 CR_SetActive(const ControllerOps* ops);
const ControllerOps* CR_GetActive(void);

uint8_t CR_ActiveNumInputs(void);
uint8_t CR_ActiveNumOutputs(void);

/* One-time handshake: serves GET_SPEC and replies with header+tokens. */
void CR_HandshakeOnce(void);

/* Data-plane bridge: converts UART bytes <-> float arrays and dispatches. */
void CR_EvaluateBytes(const unsigned char* in_bytes, unsigned char* out_bytes);

#endif /* CONTROLLER_MANAGER_H */
