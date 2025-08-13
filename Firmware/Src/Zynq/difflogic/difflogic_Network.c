#include "difflogic_Network.h"
#include "../hw_accel_link.h"
#include "xparameters.h"
#include "xstatus.h"

/* ====== Build-time selection for DiffLG ======
   DIFFLG_USE_DMA = 1 → AXI-DMA (recommended)
   DIFFLG_USE_DMA = 0 → AXI-Lite (if you have a register wrapper)
*/

#ifndef DIFFLG_USE_DMA
#define DIFFLG_USE_DMA  0
#endif

/* AXI-Lite config for DiffLG IP if applicable */
#ifndef DIFFLG_AXIL_BASE
#define DIFFLG_AXIL_BASE     0  /* fill if you have an AXI-Lite wrapper */
#endif
#ifndef DIFFLG_AXIL_REG_CTRL
#define DIFFLG_AXIL_REG_CTRL 0
#endif
#ifndef DIFFLG_AXIL_REG_IN
#define DIFFLG_AXIL_REG_IN   0
#endif
#ifndef DIFFLG_AXIL_REG_OUT
#define DIFFLG_AXIL_REG_OUT  0
#endif

/* DMA device ID for DiffLG’s DMA instance */
#ifndef DIFFLG_DMA_DEV_ID
#define DIFFLG_DMA_DEV_ID    XPAR_HARDWARE_ACCEL_DIFFLG_AXI_DMA_0_DEVICE_ID
#endif

u32 DiffLG_Network_Init(void)
{
#if DIFFLG_USE_DMA
    HWAccel_LinkConfig cfg = {
        .iface = HWACCEL_IF_AXI_DMA,
        .u.dma = { .device_id = DIFFLG_DMA_DEV_ID }
    };
#else
    HWAccel_LinkConfig cfg = {
        .iface = HWACCEL_IF_AXILITE,
        .u.axilite = {
            .base_addr = DIFFLG_AXIL_BASE,
            .reg_ctrl  = DIFFLG_AXIL_REG_CTRL,
            .reg_in    = DIFFLG_AXIL_REG_IN,
            .reg_out   = DIFFLG_AXIL_REG_OUT
        }
    };
#endif
    return (HWAccelLink_Init(&cfg) == XST_SUCCESS) ? XST_SUCCESS : XST_FAILURE;
}

void DiffLG_Network_Evaluate(UINTPTR NetworkInputBuffAddr,
                             u32     network_input_length_in_bytes,
                             UINTPTR NetworkOutputBuffAddr,
                             u32     network_output_length_in_bytes)
{
    HWAccelLink_Evaluate((const void*)NetworkInputBuffAddr, network_input_length_in_bytes,
                         (void*)NetworkOutputBuffAddr,      network_output_length_in_bytes);
}
