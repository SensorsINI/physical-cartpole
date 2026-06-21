/*
 * Pure-C neural controller using the generic Controller API.
 * Inputs are floats in the same order as NeuralImitator:
 *   ["angleD","angle_cos","angle_sin","position","positionD","target_equilibrium","target_position"]
 *
 * We apply elementwise normalization a*x + b, then call C_Network_Evaluate(in, out).
 * Output de-normalization is identity (kept as a hook if you later need it).
 */

#include "controller_api.h"
#include "NC_C/network.h"      /* C_Network_Evaluate(float in[], float out[]) */


#define MLP_ACTIVATION_NEURONS		7 // 4bytes each
#define MLP_PREDICTION_NEURONS		1 // 4bytes each

/* --- Spec (same tokens/order as NeuralImitator) ------------------------ */
static const char* const NNC_InputNames[] = {
    "angleD", "angle_cos", "angle_sin", "position", "positionD",
    "target_equilibrium", "target_position"
};

static const ControllerSpec NNC_Spec = {
    .version   = 1,
    .n_inputs  = MLP_ACTIVATION_NEURONS,
    .n_outputs = MLP_PREDICTION_NEURONS,
    .names     = NNC_InputNames
};

static const ControllerSpec* NNC_GetSpec(void) { return &NNC_Spec; }

/* --- Normalization ----------------- */
/* Normalization for Dense-7IN-32H1-32H2-1OUT-8 (matches its normalization_vec_a/b). */
static const float c_normalize_a[MLP_ACTIVATION_NEURONS] = {
    0.04595453f, 1.00000000f, 1.00000000f, 5.21186209f, 0.82011247f, 1.00000000f, 6.31313133f
};
static const float c_normalize_b[MLP_ACTIVATION_NEURONS] = {
    0.02537525f, 0.00000000f, 0.00000000f, 0.01761615f,-0.05823207f, 0.00000000f, 0.00000000f
};

static const float c_denormalize_A[MLP_PREDICTION_NEURONS] = { 1.0f };
static const float c_denormalize_B[MLP_PREDICTION_NEURONS] = { 0.0f };

/* --- Lifecycle: no state to manage for pure-C path --------------------- */
static void NNC_Init(void)    { /* no-op */ }
static void NNC_Release(void) { /* no-op */ }

/* --- Core: float inputs → normalize → C_Network_Evaluate --------------- */
static void NNC_Evaluate(const float* in, float* out)
{
    /* Rationale: build a dense contiguous input vector with normalized features.
       This keeps the C model and the API decoupled and avoids aliasing pitfalls. */
    float c_in [MLP_ACTIVATION_NEURONS];
    float c_out[MLP_PREDICTION_NEURONS];

    for (int i = 0; i < MLP_ACTIVATION_NEURONS; ++i)
        c_in[i] = c_normalize_a[i] * in[i] + c_normalize_b[i];

    C_Network_Evaluate(c_in, c_out);

    for (int i = 0; i < MLP_PREDICTION_NEURONS; ++i) {
        float y = c_denormalize_A[i] * c_out[i] + c_denormalize_B[i];
        if (y >  1.0f) y =  1.0f;
        if (y < -1.0f) y = -1.0f;
        out[i] = y;
    }
}

/* --- Public ops table -------------------------------------------------- */
const ControllerOps NNC_Ops = {
    .spec     = NNC_GetSpec,
    .init     = NNC_Init,
    .evaluate = NNC_Evaluate,
    .release  = NNC_Release
};
