#include "HLS4ML_Network.h"
#include "../hw_accel_link.h"
#include "xparameters.h"
#include "xstatus.h"

/* ====== Build-time selection ======
   Set these macros in your project settings or keep defaults below.
   HLS4ML_USE_DMA = 0 → AXI-Lite registers
   HLS4ML_USE_DMA = 1 → AXI-DMA (AXI-Stream)
*/

#ifndef HLS4ML_USE_DMA
#define HLS4ML_USE_DMA  0
#endif

/* AXI-Lite config (edit to match your wrapper) */
#ifndef HLS4ML_AXIL_BASE
#define HLS4ML_AXIL_BASE      XPAR_HARDWARE_ACCEL_MLP_AXI_LITE_WRAPPER_0_BASEADDR
#endif
#ifndef HLS4ML_AXIL_REG_CTRL
#define HLS4ML_AXIL_REG_CTRL  0x00
#endif
#ifndef HLS4ML_AXIL_REG_IN
#define HLS4ML_AXIL_REG_IN    0x04
#endif
#ifndef HLS4ML_AXIL_REG_OUT
#define HLS4ML_AXIL_REG_OUT   0x08
#endif

/* DMA device ID (edit to your DMA instance for HLS4ML) */
#ifndef HLS4ML_DMA_DEV_ID
#define HLS4ML_DMA_DEV_ID     XPAR_HARDWARE_ACCEL_HLS4ML_AXI_DMA_0_DEVICE_ID
#endif

u32 HLS4ML_Network_Init(void)
{
#if HLS4ML_USE_DMA
    HWAccel_LinkConfig cfg = {
        .iface = HWACCEL_IF_AXI_DMA,
        .u.dma = { .device_id = HLS4ML_DMA_DEV_ID }
    };
#else
    HWAccel_LinkConfig cfg = {
        .iface = HWACCEL_IF_AXILITE,
        .u.axilite = {
            .base_addr = HLS4ML_AXIL_BASE,
            .reg_ctrl  = HLS4ML_AXIL_REG_CTRL,
            .reg_in    = HLS4ML_AXIL_REG_IN,
            .reg_out   = HLS4ML_AXIL_REG_OUT
        }
    };
#endif
    return (HWAccelLink_Init(&cfg) == XST_SUCCESS) ? XST_SUCCESS : XST_FAILURE;
}

void HLS4ML_Network_Evaluate(UINTPTR NetworkInputBuffAddr,
                             u32     network_input_length_in_bytes,
                             UINTPTR NetworkOutputBuffAddr,
                             u32     network_output_length_in_bytes)
{
    HWAccelLink_Evaluate((const void*)NetworkInputBuffAddr, network_input_length_in_bytes,
                         (void*)NetworkOutputBuffAddr,      network_output_length_in_bytes);
}
