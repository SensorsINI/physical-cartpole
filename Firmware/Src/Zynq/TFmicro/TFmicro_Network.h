#ifndef TFMICRO_NETWORK_H_
#define TFMICRO_NETWORK_H_

#define TF_MICRO

#define TF_MICRO_ARENA_SIZE 12000
#define TF_MICRO_NUMBER_OF_INPUTS 7
#define TF_MICRO_NUMBER_OF_OUTPUTS 1

void TFmicro_Network_Init();
void TFmicro_Network_Evaluate(float * inputs, float * outputs);

#endif /* TFMICRO_NETWORK_H_ */
