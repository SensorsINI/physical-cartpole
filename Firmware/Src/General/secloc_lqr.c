/*
 * Legacy monolithic SecLoc + LQR controller (single-output) using the generic Controller API.
 * The SecLoc event gate (log-ratio criterion on |angle| and |position - target|) is fused
 * into the controller: the LQR command is recomputed only on gate events, otherwise the
 * previous command is replayed.
 *
 *   state = [ position - target_position,  positionD,  angle,  angleD ]^T
 *   Q = -K · state, then clip to [-1, 1].
 *
 * For the modular gate + inner-controller architecture see secloc_controller.c (SECLOC_Ops).
 * PC-side twin: Control_Toolkit_ASF/Controllers/controller_secloc_lqr.py ('secloc-lqr').
 */

#include "secloc_lqr.h"
#include <math.h>

/* ---- Spec: declare exact input tokens in wire order. ---- */
static const char* const SECLOC_LQR_InputNames[] = {
    "position", "positionD", "angle", "angleD", "target_position"
};

static const ControllerSpec SECLOC_LQR_Spec = {
    .version   = 1,
    .n_inputs  = 5,
    .n_outputs = 1,
    .names     = SECLOC_LQR_InputNames
};

/* ---- Gains (float-precision constants). ----
 * Computed by controller_secloc_lqr.py; alternative set for Q=[20000,1,1,1], R=2000:
 * -3.16227766, -5.38397176, 15.74078116, 1.92755949 */
static const float K0 = -0.99999994f;    /* from -0.9999999999999905  -> float */
static const float K1 = -4.08279276f;    /* from -4.082792666616755   -> float */
static const float K2 =  10.16550636f;   /* from  10.165506421041542  -> float */
static const float K3 =  1.63372719f;    /* from  1.6337271346217523  -> float */

static const float ang_dead_band = 0.0f;
static const float pos_dead_band = 0.0f;
static const float log_base = 1.05f;

/* ---- Lifecycle hooks (kept trivial). ---- */
static void SECLOC_LQR_Init(void)    { /* nothing to set up at the program start */ }
static void SECLOC_LQR_Release(void) { /* nothing to free at the program end  */ }

/* ---- Core: evaluate() implements the SecLoc gate + Q = -K·[p-tp, pd, a, ad]. ---- */
static void SECLOC_LQR_Evaluate(const float* in, float* out)
{
    /* in[] matches SECLOC_LQR_InputNames order */
    const float p   = in[0];
    const float pd  = in[1];
    const float a   = in[2];
    const float ad  = in[3];
    const float tp  = in[4];

    static unsigned long long calls = 0;
    static unsigned long long spikes = 0;
    static float Q = 0;
    static int has_init = 0;            /* Initialization flag. */

    static float ang_last_shift;        /* Last event angle shift. */
    static float pos_last_shift = 0;    /* Last event position shift. */

    float ang_shift = a;                /* Current angle shift (error). */
    float ang_shift_sign = 1;           /* Current angle shift sign. */
    float pos_shift = p - tp;           /* Current position shift (error). */
    float pos_shift_sign = 1;           /* Current position shift sign. */

    if (ang_shift < 0){
        ang_shift_sign = -1;
        ang_shift = -ang_shift;
    }
    if (pos_shift < 0){
        pos_shift_sign = -1;
        pos_shift = -pos_shift;
    }

    if (has_init == 0){                 /* Checks if controller has been initialized. */
        has_init = 1;
        ang_last_shift = 0.0001f;
        pos_last_shift = 0.0001f;
    }
    if ((ang_shift > ang_dead_band) && (ang_last_shift != 0)){    /* ang_last_shift cant be 0. */
        float ang_ratio_inc = ang_shift/ang_last_shift;
        float ang_ratio_dec = 1.0f/ang_ratio_inc;
        if ((ang_ratio_inc >= log_base) || (ang_ratio_dec >= log_base)){
            ang_last_shift = ang_shift;
            Q = -(K0*pos_shift*pos_shift_sign + K1*pd + K2*ang_shift*ang_shift_sign + K3*ad);
            spikes++;
        }
    }
    else if((pos_shift > pos_dead_band) && (pos_last_shift != 0)){
        float pos_ratio_inc = pos_shift/pos_last_shift;
        float pos_ratio_dec = 1.0f/pos_ratio_inc;
        if ((pos_ratio_inc >= log_base) || (pos_ratio_dec >= log_base)){
            pos_last_shift = pos_shift;
            Q = -(K0*pos_shift*pos_shift_sign + K1*pd + K2*ang_shift*ang_shift_sign + K3*ad);
            spikes++;
        }
    }
    if (Q >  1.0f) Q =  1.0f;
    if (Q < -1.0f) Q = -1.0f;
    calls++;
    (void)calls; (void)spikes;
    out[0] = Q;
}

/* ---- Public vtable ---- */
static const ControllerSpec* SECLOC_LQR_GetSpec(void) { return &SECLOC_LQR_Spec; }

const ControllerOps SECLOC_LQR_Ops = {
    .spec     = SECLOC_LQR_GetSpec,
    .init     = SECLOC_LQR_Init,
    .evaluate = SECLOC_LQR_Evaluate,
    .release  = SECLOC_LQR_Release
};
