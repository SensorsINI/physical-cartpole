#include "neural_imitator.h"

#include "hw_platform_config.h"

#include "buttons_and_switches.h"


#include "fixed_point.hpp"

#if HW_HAS_EDGEDRNN
#define EdgeDRNN
#endif

#ifdef EdgeDRNN
#include "EdgeDRNN/EdgeDRNN_Network.h"
#endif

#if HW_HAS_HLS4ML
#define HLS4ML
#endif

#ifdef HLS4ML
#include "HLS4ML/HLS4ML_Network.h"
#endif

#if HW_HAS_DIFFLG
#define DIFFLG
#endif

#ifdef DIFFLG
#include "DiffLG/DiffLG_Network.h"
#include "difflogic/difflg_weights.h"
#endif

/******************** make this module a float-based controller **********/
#include "../controller_api.h"
#include <string.h>   /* memcpy */

/* Wire-order input names the PC will use; keep short, ASCII. */
static const char* const kNN_InputNames[] = {
    "angleD", "angle_cos", "angle_sin", "position", "positionD",
    "target_equilibrium", "target_position"
};

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(kNN_InputNames)/sizeof(kNN_InputNames[0]) == MLP_ACTIVATION_NEURONS,
               "neural_imitator: names count != MLP_ACTIVATION_NEURONS");
#endif

static const ControllerSpec kNN_Spec = {
    .version   = 1,
    .n_inputs  = MLP_ACTIVATION_NEURONS,
    .n_outputs = MLP_PREDICTION_NEURONS,
    .names     = kNN_InputNames
};

static const ControllerSpec* nn_spec(void) { return &kNN_Spec; }
static void nn_init(void)    { Neural_Imitator_Init(); }
static void nn_release(void) { Neural_Imitator_ReleaseResources(); }

/* Controllers speak floats; the legacy core consumes raw bytes.
 * We bridge locally using memcpy to avoid aliasing/alignment UB. */
static void nn_evaluate(const float* in, float* out)
{
    unsigned char in_b [MLP_ACTIVATION_NEURONS * 4];
    unsigned char out_b[MLP_PREDICTION_NEURONS * 4];

    for (uint8_t i = 0; i < kNN_Spec.n_inputs;  ++i) memcpy(&in_b [i*4], &in [i], 4);
    Neural_Imitator_Evaluate(in_b, out_b);
    for (uint8_t i = 0; i < kNN_Spec.n_outputs; ++i) memcpy(&out[i],    &out_b[i*4], 4);
}

/* Exported ops object used by the runtime. */
const ControllerOps NeuralImitator_Ops = {
    .spec    = nn_spec,
    .init     = nn_init,
    .evaluate = nn_evaluate,
    .release  = nn_release
};


#define NETWORKS_SWITCH_NUMBER	3

#define MEM_BASE_ADDR		0x01000000

#define TX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00100000)
#define RX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00300000)

// hpf_20250918_1
//float hls_normalize_a[] = {0.06736747,1.00000000,1.00000000,5.05050516,0.92031026,1.00000000,5.05050516
//};
//float hls_normalize_b[] = {0.00000000,0.00000000,0.00000000,0.00000000,0.00000000,0.00000000,0.00000000
//};
//float hls_denormalize_A[] = {1.0};
//float hls_denormalize_B[] = {0.0};

// Normalization Cartpole hpf_v2024_x3232_12_2_v1
float hls_normalize_a[] = {0.04595453,1.00000000,1.00000000,5.21186209,0.82011247,1.00000000,6.31313133
};
float hls_normalize_b[] = {0.02537525,0.00000000,0.00000000,0.01761615,-0.05823207,0.00000000,0.00000000
};
float hls_denormalize_A[] = {1.0};
float hls_denormalize_B[] = {0.0};


//// Normalization Cartpole hpf_v2024_cpp_x3232_12_2_v2
//float hls_normalize_a[] = {0.05373850,1.00000000,1.00000000,5.44010401,0.86096680,1.00000000,6.31313133
//};
//float hls_normalize_b[] = {-0.07883823,0.00000000,0.00000000,0.01648343,-0.01449436,0.00000000,0.00000000
//};
//float hls_denormalize_A[] = {1.0};
//float hls_denormalize_B[] = {0.0};

// Normalization Cartpole hpf_v2024_cpp_x3232_14_3_short_v1
//float hls_normalize_a[] = {0.00332930,1.00000000,1.00000000,5.05050516,0.18420650,1.00000000,5.05050516
//};
//float hls_normalize_b[] = {0.00000000,0.00000000,0.00000000,0.00000000,0.00000000,0.00000000,0.00000000
//};
//float hls_denormalize_A[] = {1.0};
//float hls_denormalize_B[] = {0.0};
//float norm_vect[] = {0.05440483, 1.0, 1.0, 5.05050516, 0.88866770, 1.0, 5.05050516};


// Normalization f1t 31
//float hls_normalize_a[] = {3.10077524,2.03045702,1.03896105,0.62952471,0.43591982,0.32637075,0.25913447,0.21468441,0.18336849,1.39860129,1.39860129,0.68027210,0.44179368,0.32976091,0.26413101,0.22121446,0.19166268,0.16701461,0.14775415,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.12087514,0.14740567,1.26502204,2.23214269
//};
//float hls_normalize_b[] = {-2.16899228,-2.69137049,-1.71376622,-1.14667928,-0.83347863,-0.63315928,-0.50323915,-0.41713184,-0.35711008,0.06713283,0.06713283,0.03877544,0.02628672,0.01665294,0.00449026,0.00940156,0.01102066,0.01244247,0.01019502,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,0.03046048,-1.09772992,-0.05249846,0.00000000
//};
//float hls_denormalize_A[] = {0.78250003,8.06599998
//};
//float hls_denormalize_B[] = {-0.01349998,7.27099991
//};

// EdgeDRNN input buffer
short* edgedrnn_stim; //[] = {131,256,8,36,2,256,14,0};


void Neural_Imitator_Init()
{

#ifdef HLS4ML
	HLS4ML_Network_Init();
#endif

#ifdef EdgeDRNN
	EdgeDRNN_Network_Init();
#endif

}

#ifdef DIFFLG
    DiffLG_Network_Init();
#endif


void Neural_Imitator_Evaluate(unsigned char * network_input_buffer, unsigned char * network_output_buffer)
{

	int32_t *TxBufferPtr;
	int32_t *RxBufferPtr;

	TxBufferPtr = (int32_t *) TX_BUFFER_BASE;
	RxBufferPtr = (int32_t *) RX_BUFFER_BASE;

	float actv_floating_point, predic_floating_point;
	int32_t actv_fixed_point_32;
	short actv_fixed_point_16;
	int32_t predic_fixed_point_32;

	NeuralNetworkType active_network;

    // Determine active network based on the switch state
    if (Switch_GetState(NETWORKS_SWITCH_NUMBER)) {
        active_network = SELECTED_NETWORK_UP;
    } else {
        active_network = SELECTED_NETWORK_DOWN;
    }


    switch (active_network) {
        case NETWORK_EDGEDRNN:
    #ifdef EdgeDRNN
            {
                // Use EdgeDRNN accelerator

                for (int neuron_idx = 0; neuron_idx < MLP_ACTIVATION_NEURONS; neuron_idx++) {
                    actv_floating_point = *((float*)&network_input_buffer[neuron_idx * DATA_WORD_BYTES]);
                    actv_fixed_point_16 = float_to_fixed_16(actv_floating_point, 8);
                    edgedrnn_stim[neuron_idx] = actv_fixed_point_16;
                }
                edgedrnn_stim[7] = 0; // FIXME: This is probably just setting the target equilibrium or position to 0

                predic_floating_point = EdgeDRNN_Network_Evaluate((short*)(edgedrnn_stim));

                for (int neuron_idx = 0; neuron_idx < MLP_PREDICTION_NEURONS; neuron_idx++) {
                    *((float*)&network_output_buffer[neuron_idx * DATA_WORD_BYTES]) = predic_floating_point;
                }
            }
    #endif
            break;

        case NETWORK_HLS4ML:
    #ifdef HLS4ML
            {
                // Use MLP accelerator

                for (int neuron_idx = 0; neuron_idx < MLP_ACTIVATION_NEURONS; neuron_idx++) {
                    actv_floating_point = *((float*)&network_input_buffer[neuron_idx * DATA_WORD_BYTES]);
                    actv_floating_point = hls_normalize_a[neuron_idx] * actv_floating_point + hls_normalize_b[neuron_idx];
                    actv_fixed_point_32 = float_to_fixed_32(actv_floating_point, MLP_TOTAL_BITS_PER_VARIABLE - MLP_INTEGER_PLUS_SIGN_BITS_PER_VARIABLE);
                    TxBufferPtr[neuron_idx] = actv_fixed_point_32;
                }

                HLS4ML_Network_Evaluate((UINTPTR)TxBufferPtr, NETWORK_INPUT_SIZE_IN_BYTES, (UINTPTR)RxBufferPtr, NETWORK_OUTPUT_SIZE_IN_BYTES);

                for (int neuron_idx = 0; neuron_idx < MLP_PREDICTION_NEURONS; neuron_idx++) {
                    predic_fixed_point_32 = extend_sign_32(RxBufferPtr[neuron_idx], MLP_TOTAL_BITS_PER_VARIABLE - 1);
                    predic_floating_point = fixed_to_float_32(predic_fixed_point_32, MLP_TOTAL_BITS_PER_VARIABLE - MLP_INTEGER_PLUS_SIGN_BITS_PER_VARIABLE);
                    predic_floating_point = hls_denormalize_A[neuron_idx] * predic_floating_point + hls_denormalize_B[neuron_idx];
                    *((float*)&network_output_buffer[neuron_idx * DATA_WORD_BYTES]) = predic_floating_point;
                }
            }
    #endif
            break;

        case NETWORK_DIFFLOGIC:
    #ifdef DIFFLG
            {
                // Use DiffLogic accelerator
                int tx_counter = 0;
                for (int neuron_idx = 0; neuron_idx < MLP_ACTIVATION_NEURONS; neuron_idx++)
                {
                    actv_floating_point = *((float*)&network_input_buffer[neuron_idx * DATA_WORD_BYTES]);
                    actv_floating_point = actv_floating_point * norm_vect[neuron_idx];
                    actv_floating_point = (actv_floating_point > 1.0f) ? 1.0f : (actv_floating_point < -1.0f ? -1.0f : actv_floating_point);

                    actv_floating_point = (actv_floating_point + 1.0) / 2.0;
                    float threshold = 0.0f;
                    float step = 1.0f / 100.0f;
                    for (int bits = 0; bits < 100; bits += 32)
                    {
                        int32_t value = 0;
                        for (int q = 0; q < 32; ++q, threshold += step)
                        {
                            if (actv_floating_point >= threshold)
                                value |= (1<<31);
                            value >>= 1;
                        }
                        TxBufferPtr[tx_counter++] = value;
                    }
                }
                int rx_counter = 1000 / 32;
                DiffLG_Network_Evaluate((UINTPTR)TxBufferPtr, tx_counter * sizeof(int32_t),
                        (UINTPTR)RxBufferPtr, (u32)(rx_counter * sizeof(int32_t)));

                float output = linear_biases[0];
                for (size_t i = 0; i < rx_counter; ++i)
                {
                    int32_t value = RxBufferPtr[i];
                    for (int q = 0; q < 32; q++, value >>= 1)
                        if (value & 1)
                            output += linear_weight_0[i*32 + q];
                }


                output = (output > 1.0f) ? 1.0f : (output < -1.0f ? -1.0f : output);
                *((float*)&network_output_buffer[0]) = output;


                // int32_t count = 0;
                // for (size_t i = 0; i < rx_counter; ++i) {
                //     count += __builtin_popcount(RxBufferPtr[i]);
                // }
                // *((float*)&network_output_buffer[0]) = ((float)count / 500.0f) - 1.0f;
            }
    #endif
            break;
        default:
        {
            for (int neuron_idx = 0; neuron_idx < MLP_PREDICTION_NEURONS; neuron_idx++) {
                *((float*)&network_output_buffer[neuron_idx * DATA_WORD_BYTES]) = 0.0;
            }
        }
            break;
    }
}


void Neural_Imitator_ReleaseResources(){
#ifdef EdgeDRNN
	EdgeDRNN_Network_ReleaseResources();
#endif
}


