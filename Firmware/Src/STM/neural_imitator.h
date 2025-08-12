#ifndef __NEURAL_IMITATOR_H_
#define __NEURAL_IMITATOR_H_

void Neural_Imitator_Init();
void Neural_Imitator_Evaluate(unsigned char * network_input_buffer, unsigned char * network_output_buffer);
void Neural_Imitator_ReleaseResources();

#endif /*__NEURAL_IMITATOR_H_*/
