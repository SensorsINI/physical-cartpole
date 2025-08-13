#ifndef HW_ACCEL_LINK_H
#define HW_ACCEL_LINK_H

#include <stdint.h>
#include "xil_types.h"

typedef enum {
    HWACCEL_IF_AXILITE = 0,
    HWACCEL_IF_AXI_DMA = 1
} HWAccel_Interface;

typedef struct {
    uint32_t base_addr;
    uint32_t reg_ctrl;
    uint32_t reg_in;
    uint32_t reg_out;
} HWAccel_AXILiteCfg;

typedef struct {
    int device_id;   /* XPAR_* device ID of that DMA IP */
} HWAccel_AXIDmaCfg;

typedef struct {
    HWAccel_Interface iface;
    union {
        HWAccel_AXILiteCfg axilite;
        HWAccel_AXIDmaCfg  dma;
    } u;
} HWAccel_LinkConfig;

/* Init once per currently selected accelerator (call again if you switch). */
int  HWAccelLink_Init(const HWAccel_LinkConfig* cfg);

/* Blocking evaluate: send tx_bytes from tx, read rx_bytes into rx. */
void HWAccelLink_Evaluate(const void* tx, u32 tx_bytes, void* rx, u32 rx_bytes);

#endif /* HW_ACCEL_LINK_H */
