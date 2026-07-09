/*
 * Minimal LQR controller (single-output) using the generic Controller API.
 *   state = [ position - target_position,  positionD,  angle,  angleD ]^T
 *   Q = -K · state, then clip to [-1, 1].
 */

#include "lqr.h"

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
const LqrGains LQR_DefaultGains = {
    .K0 = -2.23606798f,
    .K1 = -4.55099888f,
    .K2 =  10.86611478f,
    .K3 =  1.71854851f,
};

float lqr_evaluate(float p, float pd, float a, float ad, float tp, const LqrGains* gains)
{
    const float e = p - tp;
    float Q = -(gains->K0 * e + gains->K1 * pd + gains->K2 * a + gains->K3 * ad);

    if (Q >  1.0f) Q =  1.0f;
    if (Q < -1.0f) Q = -1.0f;

    return Q;
}

static void LQR_Init(void)    { /* nothing to set up at the program start */ }
static void LQR_Release(void) { /* nothing to free at the program end  */ }

static void LQR_Evaluate(const float* in, float* out)
{
    out[0] = lqr_evaluate(in[0], in[1], in[2], in[3], in[4], &LQR_DefaultGains);
}

static const ControllerSpec* LQR_GetSpec(void) { return &LQR_Spec; }

const ControllerOps LQR_Ops = {
    .spec     = LQR_GetSpec,
    .init     = LQR_Init,
    .evaluate = LQR_Evaluate,
    .release  = LQR_Release
};
