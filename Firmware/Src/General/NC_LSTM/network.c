#include "network.h"

#include <math.h>
#include <string.h>

#if defined(__GNUC__) && !defined(NN_DISABLE_OPTIMIZATION)
#pragma GCC push_options
#pragma GCC optimize ("O3")
#endif

static float h1[QUANT_LSTM_LAYER1_UNITS];
static float c1[QUANT_LSTM_LAYER1_UNITS];
static float h2[QUANT_LSTM_LAYER2_UNITS];
static float c2[QUANT_LSTM_LAYER2_UNITS];

void QuantLSTM_Network_Reset(void)
{
    memset(h1, 0, sizeof(h1));
    memset(c1, 0, sizeof(c1));
    memset(h2, 0, sizeof(h2));
    memset(c2, 0, sizeof(c2));
}

static inline float sigmoidf_local(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

/* Keras LSTM gate order is [input, forget, candidate, output]. */
static void lstm_forward(
    const float* x,
    float* h,
    float* c,
    int input_dim,
    int units,
    const float* kernel,
    const float* recurrent,
    const float* bias)
{
    float h_previous[QUANT_LSTM_LAYER2_UNITS];
    float c_previous[QUANT_LSTM_LAYER2_UNITS];

    memcpy(h_previous, h, (unsigned int)units * sizeof(float));
    memcpy(c_previous, c, (unsigned int)units * sizeof(float));

    for (int i = 0; i < units; ++i) {
        float xi = 0.0f, xf = 0.0f, xg = 0.0f, xo = 0.0f;
        float hi = 0.0f, hf = 0.0f, hg = 0.0f, ho = 0.0f;

        for (int d = 0; d < input_dim; ++d) {
            const int base = d * (4 * units);
            xi += x[d] * kernel[base + i];
            xf += x[d] * kernel[base + units + i];
            xg += x[d] * kernel[base + 2 * units + i];
            xo += x[d] * kernel[base + 3 * units + i];
        }

        for (int u = 0; u < units; ++u) {
            const int base = u * (4 * units);
            hi += h_previous[u] * recurrent[base + i];
            hf += h_previous[u] * recurrent[base + units + i];
            hg += h_previous[u] * recurrent[base + 2 * units + i];
            ho += h_previous[u] * recurrent[base + 3 * units + i];
        }

        const float input_gate = sigmoidf_local(xi + hi + bias[i]);
        const float forget_gate = sigmoidf_local(xf + hf + bias[units + i]);
        const float candidate = tanhf(xg + hg + bias[2 * units + i]);
        const float output_gate = sigmoidf_local(xo + ho + bias[3 * units + i]);

        c[i] = forget_gate * c_previous[i] + input_gate * candidate;
        h[i] = output_gate * tanhf(c[i]);
    }
}

void QuantLSTM_Network_Evaluate(const float* inputs, float* outputs)
{
    lstm_forward(
        inputs, h1, c1,
        QUANT_LSTM_INPUT_SIZE, QUANT_LSTM_LAYER1_UNITS,
        quant_lstm_lstm1_kernel,
        quant_lstm_lstm1_recurrent_kernel,
        quant_lstm_lstm1_bias);

    lstm_forward(
        h1, h2, c2,
        QUANT_LSTM_LAYER1_UNITS, QUANT_LSTM_LAYER2_UNITS,
        quant_lstm_lstm2_kernel,
        quant_lstm_lstm2_recurrent_kernel,
        quant_lstm_lstm2_bias);

    for (int row = 0; row < QUANT_LSTM_OUTPUT_SIZE; ++row) {
        float value = quant_lstm_bias3[row];
        for (int col = 0; col < QUANT_LSTM_LAYER2_UNITS; ++col) {
            value += quant_lstm_weights3[row * QUANT_LSTM_LAYER2_UNITS + col] * h2[col];
        }
        outputs[row] = value;
    }
}

#if defined(__GNUC__) && !defined(NN_DISABLE_OPTIMIZATION)
#pragma GCC pop_options
#endif
