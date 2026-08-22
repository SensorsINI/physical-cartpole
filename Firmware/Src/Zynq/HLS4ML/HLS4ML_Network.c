#include "HLS4ML_Network.h"
#include "../hw_accel_link.h"
#include "../hw_platform_config.h"
#include "xstatus.h"

/* ====== HLS4ML Interface Auto-Selection ======
   Interface is automatically selected based on available hardware:
   - AXI4-Memory (highest priority) - if controller AXI interface exists
   - AXI-Stream (medium priority)   - if HLS4ML DMA exists  
   - AXI-Lite (lowest priority)     - if HLS4ML AXI-Lite exists
*/

/* Auto-select interface based on available hardware */
#if HW_HAS_CONTROLLER_AXI
    #define HLS4ML_INTERFACE 2  /* AXI4-Memory */
#elif HW_HAS_HLS4ML_DMA
    #define HLS4ML_INTERFACE 1  /* AXI-Stream */
#elif HW_HAS_CONTROLLER_AXILITE
    #define HLS4ML_INTERFACE 0  /* AXI-Lite */
#elif HW_HAS_SECLOC_FRONTEND
    /* The network sits behind the SecLoc frontend IP (internal AXIS link);
     * neural_imitator.c talks to secloc_frontend_link.c directly and this
     * module compiles to stubs. */
    #define HLS4ML_INTERFACE 3
#else
    #error "No HLS4ML interface hardware detected! Check hw_platform_config.h"
#endif

#if HLS4ML_INTERFACE == 3

u32 HLS4ML_Network_Init(void) { return XST_SUCCESS; }

void HLS4ML_Network_Evaluate(UINTPTR NetworkInputBuffAddr,
                             u32     network_input_length_in_bytes,
                             UINTPTR NetworkOutputBuffAddr,
                             u32     network_output_length_in_bytes)
{
    (void)NetworkInputBuffAddr;
    (void)network_input_length_in_bytes;
    (void)NetworkOutputBuffAddr;
    (void)network_output_length_in_bytes;
}

#else /* direct interfaces */

/* Configuration parameters for selected interface */
#if HLS4ML_INTERFACE == 0  /* AXI-Lite */
    #ifndef HLS4ML_AXIL_BASE
    #define HLS4ML_AXIL_BASE      HW_CONTROLLER_AXILITE_BASEADDR
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
    #define HLS4ML_DMA_DEV_ID     HW_HLS4ML_DMA_DEVICE_ID
    #endif

#elif HLS4ML_INTERFACE == 2  /* AXI4-Memory */
    #ifndef HLS4ML_AXI4_BASE
    #define HLS4ML_AXI4_BASE      HW_CONTROLLER_AXI_BASEADDR
    #endif
    #ifndef HLS4ML_AXI4_INPUT_BASE
    #define HLS4ML_AXI4_INPUT_BASE  0x010
    #endif
    #ifndef HLS4ML_AXI4_OUTPUT_BASE
    #define HLS4ML_AXI4_OUTPUT_BASE 0x200
    #endif
    /* CDMA device ID for burst transfers */
    #ifdef HWACCEL_ENABLE_CDMA
        #if HW_HAS_CDMA
            #define HLS4ML_CDMA_DEVICE_ID  HW_CDMA_DEVICE_ID
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

#endif /* HLS4ML_INTERFACE == 3 */
