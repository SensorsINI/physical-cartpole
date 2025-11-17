#ifndef HLS4ML_NETWORK_H_
#define HLS4ML_NETWORK_H_

#include "xil_types.h"

u32  HLS4ML_Network_Init(void);
void HLS4ML_Network_Evaluate(UINTPTR in_addr,  u32 in_len_bytes,
                             UINTPTR out_addr, u32 out_len_bytes);
#endif
