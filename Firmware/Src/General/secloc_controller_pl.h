#ifndef SECLOC_CONTROLLER_PL_H
#define SECLOC_CONTROLLER_PL_H

#include <stdint.h>

/*
 * Optional PL backend of the SecLoc controller (Zynq three-block SecLoc
 * chain: shell / gate / marshal). Platform code
 * (Firmware/Src/Zynq/secloc_frontend_link.c) registers the ops at boot when
 * the hardware is present; on STM32 and in the PC ctypes build nothing
 * registers and the SW path is used unconditionally.
 *
 * This module is platform-independent: it only talks to the registered ops
 * struct. All MMIO lives in the registering platform code.
 */

typedef enum {
    SECLOC_BACKEND_SW        = 0,  /* gate + inner controller on the CPU */
    SECLOC_BACKEND_PL        = 1,  /* gate + NN fused in the PL */
    SECLOC_BACKEND_PL_SHADOW = 2,  /* control from the PL; SW gate stepped in
                                      parallel, decisions compared */
} SeclocBackend;

typedef struct {
    /* One fused SecLoc step in the PL: gate decision, NN evaluation on fire,
     * zero-order hold otherwise. tick is the throttle tick (< 0 = none).
     * Returns 1 on success, 0 on transport failure (caller falls back to the
     * SW path for this step). */
    int (*evaluate)(
        float p, float pd, float a, float ad, float tp, float te,
        int32_t tick,
        float* Q, uint8_t* fired, uint8_t* gate_evaluated
    );
    void (*set_params)(
        float log_base, int32_t ref_period_ticks,
        float ang_dead_band, float pos_dead_band
    );
    void (*reset_gate)(void);
    /* Diagnostics from the most recent PL transaction (0 when unsupported). */
    uint32_t (*update_count)(void);    /* NN evaluations since gate reset */
    uint32_t (*nn_wait_cycles)(void);  /* PL cycles spent waiting for the NN */
} SeclocPlBackendOps;

/* ops must have static storage duration; pass NULL to unregister. Pushes the
 * current gate config to the PL and resets the PL gate. */
void secloc_register_pl_backend(const SeclocPlBackendOps* ops);
int secloc_pl_backend_available(void);

/* Selecting a PL backend without registered ops silently degrades to SW. */
void secloc_set_backend(SeclocBackend backend);
SeclocBackend secloc_get_backend(void);

/* Steps where SW and PL gate decisions disagreed in PL_SHADOW mode since the
 * last SECLOC init (expected to stay 0: both run the same float32 code). */
uint32_t secloc_shadow_mismatch_count(void);

/* PL diagnostics passthrough (0 when no PL backend is registered). Streamed
 * to the PC on CMD_GET_SECLOC_INFO. */
uint32_t secloc_pl_update_count(void);
uint32_t secloc_pl_nn_wait_cycles(void);

#endif /* SECLOC_CONTROLLER_PL_H */
