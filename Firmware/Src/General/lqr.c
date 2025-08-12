/*
 * Minimal LQR controller (single-output) using the generic Controller API.
 *   state = [ position - target_position,  positionD,  angle,  angleD ]^T
 *   Q = -K · state, then clip to [-1, 1].
 */

#include "controller_api.h"
#include <math.h>

/* ---- Spec: declare exact input tokens in wire order. ---- */
static const char* const LQR_InputNames[] = {
    "position", "positionD", "angle", "angleD", "target_position"
};

static const ControllerSpec LQR_Spec = {
    .version   = 1,
    .n_inputs  = 5,
    .n_outputs = 1,
    .names     = LQR_InputNames
};

/* ---- Gains (float-precision constants). ---- */
static const float K0 = -0.99999994f;    /* from -0.9999999999999905  -> float */
static const float K1 = -4.08279276f;    /* from -4.082792666616755   -> float */
static const float K2 =  10.16550636f;   /* from  10.165506421041542  -> float */
static const float K3 =  1.63372719f;    /* from  1.6337271346217523  -> float */

/* ---- Lifecycle hooks (kept trivial). ---- */
static void LQR_Init(void)    { /* nothing to set up at the program start */ }
static void LQR_Release(void) { /* nothing to free at the program end  */ }

/* ---- Core: evaluate() implements Q = -K·[p-tp, pd, a, ad]. ---- */
static void LQR_Evaluate(const float* in, float* out)
{
    /* in[] matches LQR_InputNames order */
    const float p   = in[0];
    const float pd  = in[1];
    const float a   = in[2];
    const float ad  = in[3];
    const float tp  = in[4];

    const float e   = p - tp;

    float Q = -(K0*e + K1*pd + K2*a + K3*ad);

    /* Saturate to motor command range [-1, 1]. */
    if (Q >  1.0f) Q =  1.0f;
    if (Q < -1.0f) Q = -1.0f;

    out[0] = Q;
}

/* ---- Public vtable ---- */
static const ControllerSpec* LQR_GetSpec(void) { return &LQR_Spec; }

const ControllerOps LQR_Ops = {
    .spec     = LQR_GetSpec,
    .init     = LQR_Init,
    .evaluate = LQR_Evaluate,
    .release  = LQR_Release
};
