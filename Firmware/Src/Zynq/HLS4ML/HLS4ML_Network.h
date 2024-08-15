#ifndef HLS4ML_NETWORK_H_
#define HLS4ML_NETWORK_H_

u32 HLS4ML_Network_Init();
void HLS4ML_Network_Evaluate(UINTPTR NetworkInputBuffAddr, u32 network_input_length_in_bytes,
							 UINTPTR NetworkOutputBuffAddr, u32 network_output_length_in_bytes);


#endif /* HLS4ML_NETWORK_H_ */
