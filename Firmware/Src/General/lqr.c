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

/* ---- Gains for Q=[10000, 1, 1, 1], R=2000 (float-precision constants). ----
 * Must match Driver/CartPoleSimulation/Control_Toolkit_ASF/config_controllers.yml
 * (lqr: Q, R) so the on-chip and Python LQR are the same controller. */
static const float K0 = -2.23606798f;    /* from -2.2360679774997900  -> float */
static const float K1 = -4.55099888f;    /* from -4.5509988800000000  -> float */
static const float K2 =  10.86611478f;   /* from  10.866114780000000  -> float */
static const float K3 =  1.71854851f;    /* from  1.7185485100000000  -> float */

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
