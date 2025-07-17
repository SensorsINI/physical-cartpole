#ifndef __NEURAL_IMITATOR_H_
#define __NEURAL_IMITATOR_H_


typedef enum {
	NETWORK_NONE,
    NETWORK_HLS4ML,
    NETWORK_DIFFLOGIC,
    NETWORK_EDGEDRNN,
    NETWORK_C,
} NeuralNetworkType;

#define SELECTED_NETWORK_UP NETWORK_DIFFLOGIC;  // Assign to switch UP position
#define SELECTED_NETWORK_DOWN NETWORK_DIFFLOGIC;  // Assign to switch DOWN position


#define MLP_TOTAL_BITS_PER_VARIABLE	12
#define MLP_INTEGER_PLUS_SIGN_BITS_PER_VARIABLE	2

#define MLP_ACTIVATION_NEURONS		7 // 4bytes each
#define MLP_PREDICTION_NEURONS		1 // 4bytes each
#define DATA_WORD_BYTES				4

#define NETWORK_INPUT_SIZE_IN_BYTES		(MLP_ACTIVATION_NEURONS * DATA_WORD_BYTES)
#define NETWORK_OUTPUT_SIZE_IN_BYTES		(MLP_PREDICTION_NEURONS * DATA_WORD_BYTES)

#define NET_INPUT_BITS   698
#define NET_OUTPUT_BITS 1000

// handy derived macros
#define NET_INPUT_WORDS  ((NET_INPUT_BITS  + 31) / 32)   // → 22
#define NET_OUTPUT_WORDS ((NET_OUTPUT_BITS + 31) / 32)   // → 32

void Neural_Imitator_Init();
void Neural_Imitator_Evaluate(unsigned char * network_input_buffer, unsigned char * network_output_buffer);
void Neural_Imitator_ReleaseResources();

float neural_imitator_cartpole_step(float angle, float angleD, float angle_cos, float angle_sin, float position, float positionD, float target_equilibrium, float target_position, float time);


#endif /*__NEURAL_IMITATOR_H_*/
