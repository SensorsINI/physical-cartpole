/*
 * PL backend module of the SecLoc controller.
 *
 * Owns the registered SeclocPlBackendOps, the backend selection and the
 * PL-side counters (shadow mismatches, faults, diagnostics passthrough).
 * Platform-independent: all MMIO lives in the code that registers the ops
 * (Firmware/Src/Zynq/secloc_frontend_link.c). On platforms without a PL
 * SecLoc chain this file compiles to inert code holding a NULL pointer and
 * the backend stays SECLOC_BACKEND_SW.
 *
 * No fallback policy: the backend setting is a request, never silently
 * degraded. When a PL backend is requested and the PL block is absent or a
 * transaction fails, the step is a fault: the controller outputs zero force
 * and flags it, instead of papering over the failure with the SW
 * implementation. A dead cart is obvious even without a PC attached; the
 * telemetry flag and fault counter explain why once one is connected.
 */

#include "secloc_controller_pl.h"
#include "secloc_controller_internal.h"
#include "secloc_defaults.h"
#include "secloc_logic.h"

static const SeclocPlBackendOps* secloc_pl = 0;
/* The requested backend, applied from boot. Deliberately NOT gated on a PL
 * backend having registered: requesting PL without the hardware makes every
 * step fault (zero force) rather than silently running SW. */
static SeclocBackend secloc_backend = SECLOC_DEFAULT_BACKEND;
static uint32_t secloc_shadow_mismatches = 0;
static uint32_t secloc_pl_faults = 0;

static void secloc_pl_push_config(const SeclocConfig* config)
{
    secloc_pl->set_params(
        config->log_base,
        config->ref_period_ticks,
        config->ang_dead_band,
        config->pos_dead_band
    );
}

void secloc_register_pl_backend(const SeclocPlBackendOps* ops)
{
    secloc_pl = ops;
    if (secloc_pl) {
        secloc_pl_push_config(secloc_controller_config());
        secloc_pl->reset_gate();
    }
}

int secloc_pl_backend_available(void)
{
    return secloc_pl != 0;
}

void secloc_set_backend(SeclocBackend backend)
{
    secloc_backend = backend;
}

SeclocBackend secloc_get_backend(void)
{
    return secloc_backend;
}

uint32_t secloc_shadow_mismatch_count(void)
{
    return secloc_shadow_mismatches;
}

uint32_t secloc_pl_fault_count(void)
{
    return secloc_pl_faults;
}

uint32_t secloc_pl_update_count(void)
{
    return (secloc_pl && secloc_pl->update_count) ? secloc_pl->update_count() : 0u;
}

uint32_t secloc_pl_nn_wait_cycles(void)
{
    return (secloc_pl && secloc_pl->nn_wait_cycles) ? secloc_pl->nn_wait_cycles() : 0u;
}

/* --- internal interface towards secloc_controller.c ---------------------- */

int secloc_pl_active(void)
{
    return secloc_backend != SECLOC_BACKEND_SW;
}

int secloc_pl_step(
    SeclocState* state,
    const SeclocConfig* config,
    float p, float pd, float a, float ad, float tp, float te, float time,
    float* Q, uint8_t* fired, uint8_t* gate_evaluated
)
{
    int32_t tick = -1;

    /* PL backend requested but the PL block never registered (bitstream
     * without the IP, failed boot probe): every step is a fault. */
    if (secloc_pl == 0) {
        secloc_pl_faults++;
        return 0;
    }

    if (config->time_quantum_s > 0.0f) {
        tick = secloc_tick_from_time(time, config->time_quantum_s);
    }

    if (!secloc_pl->evaluate(p, pd, a, ad, tp, te, tick, Q, fired, gate_evaluated)) {
        secloc_pl_faults++;
        return 0;
    }

    if (secloc_backend == SECLOC_BACKEND_PL_SHADOW) {
        /* Step the SW gate on the same inputs (no inner controller run) and
         * count decision disagreements; both sides execute the same float32
         * gate code, so any mismatch flags a real integration bug. */
        int sw_fired = secloc_should_sample(
            state, config, p, pd, a, ad, tp, te, time);
        if ((sw_fired != 0) != (*fired != 0)) {
            secloc_shadow_mismatches++;
        }
    }

    return 1;
}

void secloc_pl_notify_config(const SeclocConfig* config)
{
    if (secloc_pl) {
        secloc_pl_push_config(config);
    }
}

void secloc_pl_notify_init(const SeclocConfig* config)
{
    secloc_shadow_mismatches = 0;
    secloc_pl_faults = 0;
    if (secloc_pl) {
        secloc_pl_push_config(config);
        secloc_pl->reset_gate();
    }
}
