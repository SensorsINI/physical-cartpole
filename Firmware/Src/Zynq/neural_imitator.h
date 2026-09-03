#ifndef __NEURAL_IMITATOR_H_
#define __NEURAL_IMITATOR_H_

#include <stdint.h>

typedef enum {
	NETWORK_NONE,
    NETWORK_HLS4ML,
    NETWORK_DIFFLOGIC,
    NETWORK_EDGEDRNN,
} NeuralNetworkType;

#define SELECTED_NETWORK_UP NETWORK_HLS4ML  // Assign to switch UP position
#define SELECTED_NETWORK_DOWN NETWORK_HLS4ML  // Assign to switch DOWN position


#define MLP_TOTAL_BITS_PER_VARIABLE	14  /* IROS short pole hls4ml; Dense-8 was 12/2 */
#define MLP_INTEGER_PLUS_SIGN_BITS_PER_VARIABLE	3

#define MLP_ACTIVATION_NEURONS		7 // 4bytes each
#define MLP_PREDICTION_NEURONS		1 // 4bytes each
#define DATA_WORD_BYTES				4

#define NETWORK_INPUT_SIZE_IN_BYTES		(MLP_ACTIVATION_NEURONS * DATA_WORD_BYTES)
#define NETWORK_OUTPUT_SIZE_IN_BYTES		(MLP_PREDICTION_NEURONS * DATA_WORD_BYTES)

/******************** controller API exposure ****************************/
#include "controller_api.h"

/* This controller can now be used via the generic runtime. */
extern const ControllerOps NeuralImitator_Ops;

#endif /*__NEURAL_IMITATOR_H_*/
