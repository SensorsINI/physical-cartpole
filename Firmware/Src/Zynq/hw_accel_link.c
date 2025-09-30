#include "hw_accel_link.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xaxidma.h"
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
} HWAccel_State;

static HWAccel_State G = {0};

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
        u32 timeout = 1000000;  /* prevent infinite loop */
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

        Xil_DCacheFlushRange((INTPTR)tx, tx_bytes);

        // Clear control register first
        Xil_Out32(G.axi4mem.base_addr + 0x0, 0x0u);

        // Write inputs to input memory region (0x010 + i*4)
        // The VHDL interface expects the value in the lower data_bits of each 32-bit word
        for (u32 i = 0; i < nin; ++i) {
            // Extract lower data_bits from the 32-bit fixed-point input
            u32 input_masked = ptx[i] & mask;
            Xil_Out32(G.axi4mem.base_addr + G.axi4mem.input_base + i*4, input_masked);
        }

        // Start computation (set bit 0 in control register)
        Xil_Out32(G.axi4mem.base_addr + 0x0, 0x1u);

        // Wait for done (poll bit 1 in control register)
        u32 timeout = 1000000;  // Add timeout to prevent infinite loop
        while (((Xil_In32(G.axi4mem.base_addr + 0x0) & 0x2u) == 0u) && (timeout > 0)) { 
            timeout--;
        }

        // Read outputs from output memory region (0x200 + i*4)
        // The VHDL interface provides data_bits in the lower bits
        for (u32 i = 0; i < nout; ++i) {
            u32 output_raw = Xil_In32(G.axi4mem.base_addr + G.axi4mem.output_base + i*4);
            // Sign-extend variable-width output to 32-bit for compatibility
            if (data_bits == 32u) {
                prx[i] = output_raw;  // already full width
            } else if (output_raw & (1u << sign_bit)) {
                prx[i] = output_raw | (~mask);  // Sign-extend
            } else {
                prx[i] = output_raw & mask;     // Zero-extend
            }
        }

        // Clear control register to reset FSM
        Xil_Out32(G.axi4mem.base_addr + 0x0, 0x0u);

        Xil_DCacheInvalidateRange((INTPTR)rx, rx_bytes);
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
