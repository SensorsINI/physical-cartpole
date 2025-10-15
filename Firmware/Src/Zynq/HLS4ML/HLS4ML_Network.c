#include "HLS4ML_Network.h"
#include "../hw_accel_link.h"
#include "xparameters.h"
#include "xstatus.h"

/* ====== HLS4ML Interface Auto-Selection ======
   Interface is automatically selected based on available hardware:
   - AXI4-Memory (highest priority) - if XPAR_HARDWARE_ACCEL_MLP_AXI_INTERFACE_0_BASEADDR exists
   - AXI-Stream (medium priority)   - if XPAR_HARDWARE_ACCEL_HLS4ML_AXI_DMA_0_DEVICE_ID exists  
   - AXI-Lite (lowest priority)     - if XPAR_HARDWARE_ACCEL_MLP_AXI_LITE_INTERFA_0_BASEADDR exists
*/

/* Auto-select interface based on available hardware */
#ifdef XPAR_HARDWARE_ACCEL_MLP_AXI_INTERFACE_0_BASEADDR
    #define HLS4ML_INTERFACE 2  /* AXI4-Memory */
#elif defined(XPAR_HARDWARE_ACCEL_HLS4ML_AXI_DMA_0_DEVICE_ID)
    #define HLS4ML_INTERFACE 1  /* AXI-Stream */
#elif defined(XPAR_HARDWARE_ACCEL_MLP_AXI_LITE_INTERFA_0_BASEADDR)
    #define HLS4ML_INTERFACE 0  /* AXI-Lite */
#else
    #error "No HLS4ML interface hardware detected! Check your XPAR definitions."
#endif

/* Configuration parameters for selected interface */
#if HLS4ML_INTERFACE == 0  /* AXI-Lite */
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

#elif HLS4ML_INTERFACE == 1  /* AXI-Stream */
    #ifndef HLS4ML_DMA_DEV_ID
    #define HLS4ML_DMA_DEV_ID     XPAR_HARDWARE_ACCEL_HLS4ML_AXI_DMA_0_DEVICE_ID
    #endif

#elif HLS4ML_INTERFACE == 2  /* AXI4-Memory */
    #ifndef HLS4ML_AXI4_BASE
    #define HLS4ML_AXI4_BASE      XPAR_HARDWARE_ACCEL_MLP_AXI_INTERFACE_0_BASEADDR
    #endif
    #ifndef HLS4ML_AXI4_INPUT_BASE
    #define HLS4ML_AXI4_INPUT_BASE  0x010
    #endif
    #ifndef HLS4ML_AXI4_OUTPUT_BASE
    #define HLS4ML_AXI4_OUTPUT_BASE 0x200
    #endif
    /* CDMA device ID for burst transfers */
    #ifdef HWACCEL_ENABLE_CDMA
        #ifdef XPAR_HARDWARE_ACCEL_AXI_CDMA_0_DEVICE_ID
            #define HLS4ML_CDMA_DEVICE_ID  XPAR_HARDWARE_ACCEL_AXI_CDMA_0_DEVICE_ID
        #else
            #error "HWACCEL_ENABLE_CDMA is defined but no CDMA hardware found! Check your Vivado design or disable HWACCEL_ENABLE_CDMA in hw_accel_link.h"
        #endif
    #else
        #define HLS4ML_CDMA_DEVICE_ID  -1  /* CDMA disabled by user */
    #endif
#endif

u32 HLS4ML_Network_Init(void)
{
    HWAccel_LinkConfig cfg;
    
#if HLS4ML_INTERFACE == 1  /* AXI-Stream */
    cfg.iface = HWACCEL_IF_AXI_DMA;
    cfg.u.dma.device_id = HLS4ML_DMA_DEV_ID;
    
#elif HLS4ML_INTERFACE == 2  /* AXI4-Memory */
    cfg.iface = HWACCEL_IF_AXI4_MEM;
    cfg.u.axi4mem.base_addr   = HLS4ML_AXI4_BASE;
    cfg.u.axi4mem.input_base  = HLS4ML_AXI4_INPUT_BASE;
    cfg.u.axi4mem.output_base = HLS4ML_AXI4_OUTPUT_BASE;
    cfg.u.axi4mem.cdma_device_id = HLS4ML_CDMA_DEVICE_ID;
    
#else  /* AXI-Lite (default) */
    cfg.iface = HWACCEL_IF_AXILITE;
    cfg.u.axilite.base_addr = HLS4ML_AXIL_BASE;
    cfg.u.axilite.reg_ctrl  = HLS4ML_AXIL_REG_CTRL;
    cfg.u.axilite.reg_in    = HLS4ML_AXIL_REG_IN;
    cfg.u.axilite.reg_out   = HLS4ML_AXIL_REG_OUT;
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
