/*
 * Pure-C controller for Long/quant/LSTM-7IN-64H1-64H2-1OUT-0.
 */

#include "controller_api.h"
#include "NC_LSTM/network.h"

#define LSTM_INPUTS 7
#define LSTM_OUTPUTS 1

static const char* const NNC_LSTM_InputNames[] = {
    "angleD", "angle_cos", "angle_sin", "position", "positionD",
    "target_equilibrium", "target_position"
};

static const ControllerSpec NNC_LSTM_Spec = {
    .version = 1,
    .n_inputs = LSTM_INPUTS,
    .n_outputs = LSTM_OUTPUTS,
    .names = NNC_LSTM_InputNames
};

static const float normalize_a[LSTM_INPUTS] = {
    0.00332930f, 1.00000000f, 1.00000000f, 5.05050516f,
    0.18420650f, 1.00000000f, 5.05050516f
};

static const ControllerSpec* NNC_LSTM_GetSpec(void)
{
    return &NNC_LSTM_Spec;
}

static void NNC_LSTM_Init(void)
{
    /*
     * The verified Python controller performs one inference while CartPole
     * still contains its placeholder upright state. Preserve that exact
     * recurrent initial condition before consuming physical samples.
     */
    static const float prime_input[LSTM_INPUTS] = {
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f
    };
    float ignored[LSTM_OUTPUTS];

    QuantLSTM_Network_Reset();
    QuantLSTM_Network_Evaluate(prime_input, ignored);
}

static void NNC_LSTM_Evaluate(const float* input, float* output)
{
    float normalized[LSTM_INPUTS];
    float prediction[LSTM_OUTPUTS];

    for (int i = 0; i < LSTM_INPUTS; ++i) {
        normalized[i] = normalize_a[i] * input[i];
    }

    QuantLSTM_Network_Evaluate(normalized, prediction);

    float value = prediction[0];
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    output[0] = value;
}

static void NNC_LSTM_Release(void)
{
}

const ControllerOps NNC_LSTM_Ops = {
    .spec = NNC_LSTM_GetSpec,
    .init = NNC_LSTM_Init,
    .evaluate = NNC_LSTM_Evaluate,
    .release = NNC_LSTM_Release
};
