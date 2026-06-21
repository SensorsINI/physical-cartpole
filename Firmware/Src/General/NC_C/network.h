// network.h
#ifndef NETWORK_H
#define NETWORK_H

// Python will insert the following:
//     #define IS_GRU 0 or 1
//     #define IS_LSTM 0 or 1
// For Dense networks, both are set to 0.

#define IS_GRU 0
#define IS_LSTM 0
#define INPUT_SIZE 7      // Overwritten by python
#define LAYER1_SIZE 32     // Overwritten by python (used in Dense mode)
#define LAYER2_SIZE 32     // Overwritten by python (used in Dense mode)
#define LAYER3_SIZE 1     // Overwritten by python (used in Dense mode)

#define GRU1_UNITS      // Overwritten by python
#define GRU2_UNITS      // Overwritten by python

#define LSTM1_UNITS     // Overwritten by python
#define LSTM2_UNITS     // Overwritten by python

// Declare the function and global variables
void C_Network_Evaluate(float* inputs, float* outputs);

#if IS_GRU
void InitializeGRUStates(void);
#endif

#if IS_LSTM
void InitializeLSTMStates(void);
#endif

// Other necessary declarations if required
extern const float weights1[];
extern const float bias1[];
extern const float weights2[];
extern const float bias2[];
extern const float weights3[];
extern const float bias3[];

// These exist only for GRU networks (two GRU layers + final Dense).
// They will be empty if IS_GRU=0, but we declare them anyway:
extern const float gru1_kernel[];
extern const float gru1_recurrent_kernel[];
extern const float gru1_bias[];

extern const float gru2_kernel[];
extern const float gru2_recurrent_kernel[];
extern const float gru2_bias[];

//----------------------------------------------------
// LSTM network parameters
//----------------------------------------------------
extern const float lstm1_kernel[];
extern const float lstm1_recurrent_kernel[];
extern const float lstm1_bias[];

extern const float lstm2_kernel[];
extern const float lstm2_recurrent_kernel[];
extern const float lstm2_bias[];

//----------------------------------------------------
// Initial RNN states
//----------------------------------------------------
extern const float initial_h1[];
extern const float initial_h2[];

#if IS_LSTM
extern const float initial_c1[];
extern const float initial_c2[];
#endif

#endif