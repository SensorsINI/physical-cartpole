#include "sys.h"

#include <unistd.h>
#include "platform.h"

XScuGic XScuGicInstance;

void Sleep_ms(unsigned int ms) {
    usleep(ms * 1000); // Convert milliseconds to microseconds
}

void General_Init(void){

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
