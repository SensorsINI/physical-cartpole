//TFmicro_Network.h
#ifndef TFMICRO_NETWORK_H_
#define TFMICRO_NETWORK_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TF_MICRO

#define TF_MICRO_ARENA_SIZE 12000
#define TF_MICRO_NUMBER_OF_INPUTS 7
#define TF_MICRO_NUMBER_OF_OUTPUTS 1

#include "Dense_7IN_32H1_32H2_1OUT_0_model.h"

void TFmicro_Network_Init();
void TFmicro_Network_Evaluate(float* inputs, float* outputs);  // Updated to remove num_inputs and num_outputs

#ifdef __cplusplus
}
#endif

#endif  // TFMICRO_NETWORK_H_
