#include "hw_accel_link.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xaxidma.h"

typedef struct {
    HWAccel_Interface iface;
    HWAccel_AXILiteCfg axil;
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

        Xil_DCacheFlushRange((INTPTR)tx, tx_bytes);

        for (u32 i = 0; i < nin; ++i)
            Xil_Out32(G.axil.base_addr + G.axil.reg_in, ptx[i]);

        Xil_Out32(G.axil.base_addr + G.axil.reg_ctrl, 0x1u);  /* start */

        while ((Xil_In32(G.axil.base_addr + G.axil.reg_ctrl) & 0x2u) == 0u) { /* spin */ }

        for (u32 i = 0; i < nout; ++i)
            prx[i] = Xil_In32(G.axil.base_addr + G.axil.reg_out);

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
