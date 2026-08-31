#ifndef QUANT_LSTM_NETWORK_H
#define QUANT_LSTM_NETWORK_H

#define QUANT_LSTM_INPUT_SIZE 7
#define QUANT_LSTM_LAYER1_UNITS 64
#define QUANT_LSTM_LAYER2_UNITS 64
#define QUANT_LSTM_OUTPUT_SIZE 1

void QuantLSTM_Network_Reset(void);
void QuantLSTM_Network_Evaluate(const float* inputs, float* outputs);

extern const float quant_lstm_lstm1_kernel[];
extern const float quant_lstm_lstm1_recurrent_kernel[];
extern const float quant_lstm_lstm1_bias[];
extern const float quant_lstm_lstm2_kernel[];
extern const float quant_lstm_lstm2_recurrent_kernel[];
extern const float quant_lstm_lstm2_bias[];
extern const float quant_lstm_weights3[];
extern const float quant_lstm_bias3[];

#endif /* QUANT_LSTM_NETWORK_H */
