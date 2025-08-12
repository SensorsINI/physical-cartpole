#ifndef __NEURAL_IMITATOR_H_
#define __NEURAL_IMITATOR_H_

#include <stdint.h>

typedef enum {
	NETWORK_NONE,
    NETWORK_HLS4ML,
    NETWORK_DIFFLOGIC,
    NETWORK_EDGEDRNN,
} NeuralNetworkType;

#define SELECTED_NETWORK_UP NETWORK_DIFFLOGIC  // Assign to switch UP position
#define SELECTED_NETWORK_DOWN NETWORK_DIFFLOGIC  // Assign to switch DOWN position


#define MLP_TOTAL_BITS_PER_VARIABLE	12
#define MLP_INTEGER_PLUS_SIGN_BITS_PER_VARIABLE	2

#define MLP_ACTIVATION_NEURONS		7 // 4bytes each
#define MLP_PREDICTION_NEURONS		1 // 4bytes each
#define DATA_WORD_BYTES				4

#define NETWORK_INPUT_SIZE_IN_BYTES		(MLP_ACTIVATION_NEURONS * DATA_WORD_BYTES)
#define NETWORK_OUTPUT_SIZE_IN_BYTES		(MLP_PREDICTION_NEURONS * DATA_WORD_BYTES)

/******************** controller API exposure ****************************/
#include "controller_api.h"

/* This controller can now be used via the generic runtime. */
extern const ControllerOps NeuralImitator_Ops;

void Neural_Imitator_Init();
void Neural_Imitator_Evaluate(unsigned char * network_input_buffer, unsigned char * network_output_buffer);
void Neural_Imitator_ReleaseResources(void);

#endif /*__NEURAL_IMITATOR_H_*/
