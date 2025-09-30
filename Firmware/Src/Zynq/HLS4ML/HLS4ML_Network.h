#ifndef HLS4ML_NETWORK_H_
#define HLS4ML_NETWORK_H_

#include "xil_types.h"

/* Auto-select already present in your code; keep it. */
#if HLS4ML_INTERFACE == 0  /* AXI-Lite */
  #ifndef HLS4ML_AXIL_BASE
  #define HLS4ML_AXIL_BASE      XPAR_HARDWARE_ACCEL_MLP_AXI_LITE_INTERFA_0_BASEADDR
  #endif
  #define HLS4ML_AXIL_REG_CTRL  0x00u  /* bit0=start (W), bit1=done (R) */
  #define HLS4ML_AXIL_REG_IN    0x04u  /* write next input (LSBs used)     */
  #define HLS4ML_AXIL_REG_OUT   0x08u  /* read next output (LSBs contain)  */
#endif

u32  HLS4ML_Network_Init(void);
void HLS4ML_Network_Evaluate(UINTPTR in_addr,  u32 in_len_bytes,
                             UINTPTR out_addr, u32 out_len_bytes);
#endif
