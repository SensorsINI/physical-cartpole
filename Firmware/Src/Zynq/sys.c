#include "sys.h"

#include <unistd.h>
#include "platform.h"
#include "xtime_l.h"
#include "xil_io.h"

XScuGic XScuGicInstance;

void Sleep_ms(unsigned int ms) {
    usleep(ms * 1000); // Convert milliseconds to microseconds
}

static void ensure_global_timer(void)
{
    /* Needed whenever a USE_AMP BSP skipped XTime_SetTime in xil-crt0.
     * Harmless if the timer is already running (normal single-core BSP). */
    const u32 ctrl = Xil_In32(GLOBAL_TMR_BASEADDR + GTIMER_CONTROL_OFFSET);
    if ((ctrl & 1u) == 0u) {
        XTime_SetTime(0);
    }
}

void General_Init(void){
    ensure_global_timer();

    init_platform(); // Standard from hello world example; probably not needed for zynq

    // Start interrupts

    XScuGic_Config *IntcConfig;

    // Interrupt controller driver initialization
    IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    XScuGic_CfgInitialize(&XScuGicInstance, IntcConfig, IntcConfig->CpuBaseAddress);

    // Call to interrupt setup
    Xil_ExceptionInit();

    // Register the interrupt controller handler with the exception table
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                &XScuGicInstance);

    // Enable interrupts
    Xil_ExceptionEnable();
}

void Gic_AmpEnableDistributor(void)
{
    /* Each XScuGic_CfgInitialize() calls XScuGic_Stop(), which clears DIST_EN.
     * A USE_AMP BSP then leaves the distributor off. Safe to call on a normal
     * single-core BSP: it just writes DIST_EN=1 again after the last UART init. */
    XScuGic_DistWriteReg(&XScuGicInstance, XSCUGIC_DIST_EN_OFFSET, XSCUGIC_EN_INT_MASK);
}
