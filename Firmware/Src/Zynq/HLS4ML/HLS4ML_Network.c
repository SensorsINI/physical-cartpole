#include "HLS4ML_Network.h"
#include "../hw_accel_link.h"
#include "xparameters.h"
#include "xstatus.h"

/* ====== Build-time selection ======
   Set these macros in your project settings or keep defaults below.
   HLS4ML_USE_DMA = 0 → AXI-Lite registers
   HLS4ML_USE_DMA = 1 → AXI-DMA (AXI-Stream)
   HLS4ML_USE_DMA = 2 → AXI4 Memory Interface
*/

#ifndef HLS4ML_USE_DMA
#define HLS4ML_USE_DMA  0
#endif

/* Auto-detect AXI4 interface if available */
#ifdef XPAR_HARDWARE_ACCEL_MLP_AXI_FULL_WRAPPER_0_BASEADDR
#undef HLS4ML_USE_DMA
#define HLS4ML_USE_DMA  2
#endif

/* AXI-Lite config (edit to match your wrapper) */
#ifndef HLS4ML_AXIL_BASE
#define HLS4ML_AXIL_BASE      XPAR_HARDWARE_ACCEL_MLP_AXI_LITE_INTERFA_0_BASEADDR
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

/* AXI4 Memory Interface config */
#ifndef HLS4ML_AXI4_BASE
#define HLS4ML_AXI4_BASE      XPAR_HARDWARE_ACCEL_MLP_AXI_FULL_WRAPPER_0_BASEADDR
#endif
#ifndef HLS4ML_AXI4_INPUT_BASE
#define HLS4ML_AXI4_INPUT_BASE  0x010
#endif
#ifndef HLS4ML_AXI4_OUTPUT_BASE
#define HLS4ML_AXI4_OUTPUT_BASE 0x200
#endif

u32 HLS4ML_Network_Init(void)
{
#if HLS4ML_USE_DMA == 1
    HWAccel_LinkConfig cfg = {
        .iface = HWACCEL_IF_AXI_DMA,
        .u.dma = { .device_id = HLS4ML_DMA_DEV_ID }
    };
#elif HLS4ML_USE_DMA == 2
    HWAccel_LinkConfig cfg = {
        .iface = HWACCEL_IF_AXI4_MEM,
        .u.axi4mem = {
            .base_addr   = HLS4ML_AXI4_BASE,
            .input_base  = HLS4ML_AXI4_INPUT_BASE,
            .output_base = HLS4ML_AXI4_OUTPUT_BASE
        }
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
