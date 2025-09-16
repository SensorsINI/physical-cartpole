#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller_api.h"

/* Upper bounds to allocate once (adjust conservatively if needed). */
#define MAX_INPUTS   64
#define MAX_OUTPUTS  16

/* Control-plane UART protocol shared with the PC. */
#define SERIAL_SOF     0xAA

/* Message types - unified protocol */
#define MSG_TYPE_STATE     0x01    /* State data for controller */
#define MSG_TYPE_GET_SPEC  0x02    /* Request controller specification */
#define MSG_TYPE_PING      0x03    /* Ping/keepalive */
#define MSG_TYPE_SPEC_COOKIE 0x04  /* Announce spec change (FPGA->PC) */

/* Legacy constants for backward compatibility */
#define CMD_PING       0xC0
#define CMD_GET_SPEC  0xC6
#define CMD_SPEC_COOKIE  0xC7

#define NAME_TOKEN_LEN 16   /* fixed-size ASCII tokens, NUL padded */

void                 CR_SetActive(const ControllerOps* ops);
const ControllerOps* CR_GetActive(void);

uint8_t CR_ActiveNumInputs(void);
uint8_t CR_ActiveNumOutputs(void);

/* Check if handshake protocol has been completed */
int CR_IsHandshakeComplete(void);

/* One-time (or on-demand) handshake: serves GET_SPEC and replies with header+tokens. */
void CR_HandshakeOnce(void);

/* Data-plane bridge: converts UART bytes <-> float arrays and dispatches. */
void CR_EvaluateBytes(const unsigned char* in_bytes, unsigned char* out_bytes);

/* request switching to another controller at the next safe boundary. */
void CR_RequestSwitch(const ControllerOps* new_ops);

/* emit a one-shot spec cookie if a switch was requested; returns 1 if sent. */
int  CR_SendSpecCookieIfPending(void);

/* reset handshake state to allow new handshakes */
void CR_ResetHandshakeState(void);

/* Unified message processing - new API */
int CR_ProcessMessage(void);  /* Returns 1 if message processed, 0 if no data available */
void CR_ProcessStateData(const unsigned char* data, unsigned int length);
void CR_ProcessSpecRequest(void);
void CR_ProcessPing(void);

#endif /* CONTROLLER_MANAGER_H */
