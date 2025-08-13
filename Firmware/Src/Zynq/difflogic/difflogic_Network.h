#ifndef DIFFLG_NETWORK_H
#define DIFFLG_NETWORK_H

#include "xil_types.h"

u32  DiffLG_Network_Init(void);
void DiffLG_Network_Evaluate(UINTPTR NetworkInputBuffAddr, u32 in_bytes,
                             UINTPTR NetworkOutputBuffAddr, u32 out_bytes);

#endif /* DIFFLG_NETWORK_H */
