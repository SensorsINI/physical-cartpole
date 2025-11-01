#include "hw_accel_link.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xaxidma.h"
#ifdef HWACCEL_ENABLE_CDMA
    #include "xaxicdma.h"
    /* Verify CDMA is available in BSP when requested */
    #ifndef XPAR_XAXICDMA_NUM_INSTANCES
        #error "HWACCEL_ENABLE_CDMA is defined but CDMA driver not available in BSP! Check your platform configuration or disable HWACCEL_ENABLE_CDMA in hw_accel_link.h"
    #endif
#endif
#include "neural_imitator.h"

typedef struct {
    HWAccel_Interface iface;
    HWAccel_AXILiteCfg axil;
    HWAccel_AXI4MemCfg axi4mem;
    struct {
        XAxiDma dma;
        int     inited;
        int     device_id;
    } d;
    struct {
#ifdef HWACCEL_ENABLE_CDMA
        XAxiCdma cdma;
#endif
        int      inited;
        int      device_id;     /* from axi4mem.cdma_device_id */
    } m;
} HWAccel_State;

static HWAccel_State G = {0};

/* Static buffer for CDMA input masking (cache-aligned) */
#ifdef HWACCEL_ENABLE_CDMA
static u32 cdma_tx_buf[256] __attribute__((aligned(32)));
#endif


#ifdef HWACCEL_ENABLE_CDMA
static u32 cdma_src[256] __attribute__((aligned(32)));
static u32 cdma_dst[256] __attribute__((aligned(32)));
#endif

int HWAccelLink_Init(const HWAccel_LinkConfig* cfg)
{
    if (!cfg) return XST_FAILURE;
    G.iface = cfg->iface;

    if (G.iface == HWACCEL_IF_AXILITE) {
        G.axil = cfg->u.axilite;
        Xil_Out32(G.axil.base_addr + G.axil.reg_ctrl, 0x0u);  /* clear */
        return XST_SUCCESS;
    }

    if (G.iface == HWACCEL_IF_AXI4_MEM) {
        G.axi4mem = cfg->u.axi4mem;
        Xil_Out32(G.axi4mem.base_addr + 0x0, 0x0u);  /* clear control reg */

        /* Setup AXI CDMA for burst mode */
        G.m.inited    = 0;
        G.m.device_id = G.axi4mem.cdma_device_id;
        
#ifdef HWACCEL_ENABLE_CDMA
        /* CDMA is enabled: require valid device ID */
        if (G.m.device_id < 0) {
            xil_printf("ERROR: CDMA is enabled but no device ID configured!\r\n");
            return XST_FAILURE;
        }
        
        XAxiCdma_Config* CdmaCfg = XAxiCdma_LookupConfig(G.m.device_id);
        if (!CdmaCfg) {
            xil_printf("ERROR: CDMA device ID %d not found in configuration!\r\n", G.m.device_id);
            return XST_FAILURE;
        }
        if (XAxiCdma_CfgInitialize(&G.m.cdma, CdmaCfg, CdmaCfg->BaseAddress) != XST_SUCCESS) {
            xil_printf("ERROR: Failed to initialize CDMA!\r\n");
            return XST_FAILURE;
        }
        /* Disable interrupts; we use polling */
        XAxiCdma_IntrDisable(&G.m.cdma, XAXICDMA_XR_IRQ_ALL_MASK);
        G.m.inited = 1;
#else
        /* CDMA is disabled: MMIO will be used */
        if (G.m.device_id >= 0) {
            xil_printf("WARNING: CDMA device configured but HWACCEL_ENABLE_CDMA not defined. Using MMIO.\r\n");
        }
#endif
        return XST_SUCCESS;
    }

    /* AXI-DMA */
    G.d.inited    = 0;
    G.d.device_id = cfg->u.dma.device_id;

    XAxiDma_Config* CfgPtr = XAxiDma_LookupConfig(G.d.device_id);
    if (!CfgPtr) return XST_FAILURE;
    if (XAxiDma_CfgInitialize(&G.d.dma, CfgPtr) != XST_SUCCESS) return XST_FAILURE;
    if (XAxiDma_HasSg(&G.d.dma)) return XST_FAILURE;

    XAxiDma_IntrDisable(&G.d.dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrDisable(&G.d.dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

    G.d.inited = 1;
    return XST_SUCCESS;
}

void HWAccelLink_Evaluate(const void* tx, u32 tx_bytes, void* rx, u32 rx_bytes)
{
    if (G.iface == HWACCEL_IF_AXILITE) {
        const u32* ptx = (const u32*)tx;
        u32*       prx = (u32*)rx;
        const u32 nin  = tx_bytes / 4u;
        const u32 nout = rx_bytes / 4u;
        const u32 data_bits = (MLP_TOTAL_BITS_PER_VARIABLE >= 32u) ? 32u : (u32)MLP_TOTAL_BITS_PER_VARIABLE;
        const u32 mask      = (data_bits == 32u) ? 0xFFFFFFFFu : ((1u << data_bits) - 1u);
        const u32 sign_bit  = (data_bits == 32u) ? 31u : (data_bits - 1u);

        /* Prepare for a fresh transaction: clear start & indices */
        Xil_Out32(G.axil.base_addr + G.axil.reg_ctrl, 0x00000000u);

        /* Back-to-back input writes (one word per input; LSBs carry the quantized value) */
        for (u32 i = 0; i < nin; ++i) {
            u32 input_masked = ptx[i] & mask;
            Xil_Out32(G.axil.base_addr + G.axil.reg_in, input_masked);
        }

        /* Kick computation */
        Xil_Out32(G.axil.base_addr + G.axil.reg_ctrl, 0x00000001u);  /* start=1 */

        /* Poll for done (bit1) with timeout */
        u32 timeout = 1000000u;  /* prevent infinite loop */
        while (((Xil_In32(G.axil.base_addr + G.axil.reg_ctrl) & 0x00000002u) == 0u) && (timeout > 0u)) {
            timeout--;
        }

        /* Back-to-back output reads (one word per output; LSBs valid) */
        for (u32 i = 0; i < nout; ++i) {
            u32 output_raw = Xil_In32(G.axil.base_addr + G.axil.reg_out);
            if (data_bits == 32u) {
                prx[i] = output_raw;
            } else if (output_raw & (1u << sign_bit)) {
                prx[i] = output_raw | (~mask);  /* sign-extend */
            } else {
                prx[i] = output_raw & mask;     /* zero-extend */
            }
        }

        /* Optional: disarm start, ready for the next cycle */
        Xil_Out32(G.axil.base_addr + G.axil.reg_ctrl, 0x00000000u);
        return;
    }

    if (G.iface == HWACCEL_IF_AXI4_MEM) {
        const u32* ptx = (const u32*)tx;
        u32*       prx = (u32*)rx;
        const u32 nin  = tx_bytes / 4u;
        const u32 nout = rx_bytes / 4u;
        const u32 data_bits = (MLP_TOTAL_BITS_PER_VARIABLE >= 32u) ? 32u : (u32)MLP_TOTAL_BITS_PER_VARIABLE;
        const u32 mask      = (data_bits == 32u) ? 0xFFFFFFFFu : ((1u << data_bits) - 1u);
        const u32 sign_bit  = (data_bits == 32u) ? 31u : (data_bits - 1u);

        /* Ensure CTRL[0] = 0 so that the next write of 1 generates a clean rising edge (start). */
        Xil_Out32(G.axi4mem.base_addr + 0x0, 0x00000000u);

        /* Write inputs: use CDMA if enabled, else MMIO */
#ifdef HWACCEL_ENABLE_CDMA
        int cdma_status;
        u32 err;
        /* If masking needed, use temporary buffer to avoid modifying const input */
        const void* tx_src = tx;

        if (data_bits != 32u) {
            /* Copy and mask into static buffer without modifying original */
            if (nin > 256u) {
                /* Buffer too small - fallback to MMIO for safety */
                for (u32 i = 0; i < nin; ++i) {
                    u32 input_masked = ptx[i] & mask;
                    Xil_Out32(G.axi4mem.base_addr + G.axi4mem.input_base + i*4u, input_masked);
                }
                goto skip_cdma_input;
            }
            for (u32 i = 0; i < nin; ++i) {
                cdma_tx_buf[i] = ptx[i] & mask;
            }
            tx_src = cdma_tx_buf;
        }

        Xil_DCacheFlushRange((INTPTR)tx_src, tx_bytes);

        /* Use CDMA to transfer input buffer to device */
        {
            u32 dst_addr = G.axi4mem.base_addr + G.axi4mem.input_base;

            cdma_status = XAxiCdma_SimpleTransfer(
                &G.m.cdma,
                (UINTPTR)tx_src,
                (UINTPTR)dst_addr,
                tx_bytes,
                NULL,
                NULL
            );
            if (cdma_status != XST_SUCCESS) {
                xil_printf("ERROR: CDMA input transfer failed to start with status %d\r\n", cdma_status);
            }

            while (XAxiCdma_IsBusy(&G.m.cdma)) { /* spin */ }

            /* Check for transfer errors */
            err = XAxiCdma_GetError(&G.m.cdma);
            if (err != 0u) {
                xil_printf("ERROR: CDMA input transfer error: 0x%08x\r\n", err);
                XAxiCdma_Reset(&G.m.cdma);
                while (!XAxiCdma_ResetIsDone(&G.m.cdma)) { /* wait for reset */ }
            }
        }
skip_cdma_input:
#else
        /* Write inputs via MMIO */
        for (u32 i = 0; i < nin; ++i) {
            u32 input_masked = ptx[i] & mask;
            Xil_Out32(G.axi4mem.base_addr + G.axi4mem.input_base + i*4u, input_masked);
        }
#endif

        /* Issue start: write 1 to CTRL[0] to generate the rising edge expected by the RTL. */
        Xil_Out32(G.axi4mem.base_addr + 0x0, 0x00000001u);

        /* Wait for DONE (bit1); BUSY (bit2) should remain high until outputs are packed. */
        {
            u32 timeout = 1000000u;
            while (timeout-- > 0u) {
                u32 ctrl = Xil_In32(G.axi4mem.base_addr + 0x0);

                if (ctrl & 0x00000002u) {
                    /* DONE set: outputs are valid and latched */
                    break;
                }

                if ((ctrl & 0x00000004u) == 0u) {
                    /* BUSY deasserted but DONE not set: unexpected, avoid infinite loop */
                    break;
                }
            }
        }

        /* Read outputs: use CDMA if enabled, else MMIO */
#ifdef HWACCEL_ENABLE_CDMA
        {
            int cdma_status;
            u32 err;
            u32 src_addr = G.axi4mem.base_addr + G.axi4mem.output_base;

            /* Flush rx buffer cache before DMA to avoid dirty lines overwriting DMA results */
            Xil_DCacheFlushRange((INTPTR)rx, rx_bytes);

            cdma_status = XAxiCdma_SimpleTransfer(
                &G.m.cdma,
                (UINTPTR)src_addr,
                (UINTPTR)prx,
                rx_bytes,
                NULL,
                NULL
            );
            if (cdma_status != XST_SUCCESS) {
                xil_printf("ERROR: CDMA output transfer failed to start with status %d\r\n", cdma_status);
            }

            while (XAxiCdma_IsBusy(&G.m.cdma)) { /* spin */ }

            /* Check for transfer errors */
            err = XAxiCdma_GetError(&G.m.cdma);
            if (err != 0u) {
                xil_printf("ERROR: CDMA output transfer error: 0x%08x\r\n", err);
                XAxiCdma_Reset(&G.m.cdma);
                while (!XAxiCdma_ResetIsDone(&G.m.cdma)) { /* wait for reset */ }
            }

            /* Invalidate cache to ensure CPU reads fresh DMA'd data */
            Xil_DCacheInvalidateRange((INTPTR)rx, rx_bytes);

            /* Post-process sign extension in place if needed */
            if (data_bits != 32u) {
                for (u32 i = 0; i < nout; ++i) {
                    u32 v = prx[i];
                    if (v & (1u << sign_bit)) {
                        prx[i] = v | (~mask);   /* sign-extend */
                    } else {
                        prx[i] = v & mask;      /* zero-extend */
                    }
                }
            }
        }
#else
        /* Read outputs via MMIO */
        for (u32 i = 0; i < nout; ++i) {
            u32 output_raw = Xil_In32(G.axi4mem.base_addr + G.axi4mem.output_base + i*4u);
            if (data_bits == 32u) {
                prx[i] = output_raw;
            } else if (output_raw & (1u << sign_bit)) {
                prx[i] = output_raw | (~mask);
            } else {
                prx[i] = output_raw & mask;
            }
        }
#endif

        // Clear control register to reset FSM
        Xil_Out32(G.axi4mem.base_addr + 0x0, 0x0u);
        return;
    }

    if (!G.d.inited) return;

    Xil_DCacheFlushRange((INTPTR)tx, tx_bytes);
    Xil_DCacheFlushRange((INTPTR)rx, rx_bytes);

    (void)XAxiDma_SimpleTransfer(&G.d.dma, (UINTPTR)rx, rx_bytes, XAXIDMA_DEVICE_TO_DMA);
    (void)XAxiDma_SimpleTransfer(&G.d.dma, (UINTPTR)tx, tx_bytes, XAXIDMA_DMA_TO_DEVICE);

    while (XAxiDma_Busy(&G.d.dma, XAXIDMA_DEVICE_TO_DMA) ||
           XAxiDma_Busy(&G.d.dma, XAXIDMA_DMA_TO_DEVICE)) { /* spin */ }

    Xil_DCacheInvalidateRange((INTPTR)rx, rx_bytes);
}
