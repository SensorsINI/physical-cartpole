#include "TFmicro_Network.h"

#ifdef TF_MICRO

#include "TFmicroZynqLib/tfmicrozynq.h"

// Define interpreter and tensor arena as static global variables
static tflite::MicroInterpreter* interpreter = nullptr;
static uint8_t tensor_arena[TF_MICRO_ARENA_SIZE]; // Tensor arena size from macro

void TFmicro_Network_Init() {
    using TFmicroOpResolver = tflite::MicroMutableOpResolver<2>;

    // Function to register operations
    TfLiteStatus RegisterOps(TFmicroOpResolver& op_resolver) {
        TF_LITE_ENSURE_STATUS(op_resolver.AddFullyConnected());
        TF_LITE_ENSURE_STATUS(op_resolver.AddTanh()); // Register TANH operation
        return kTfLiteOk;
    }

    // Get model from array (model_array should be defined elsewhere)
    const tflite::Model* model = ::tflite::GetModel(model_array);

    // Set up the operation resolver
    TFmicroOpResolver op_resolver;
    if (RegisterOps(op_resolver) != kTfLiteOk) {
        // Handle error
        return;
    }

    // Create interpreter instance and allocate tensors
    interpreter = new tflite::MicroInterpreter(model, op_resolver, tensor_arena, TF_MICRO_ARENA_SIZE);
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        // Handle error
        delete interpreter;
        interpreter = nullptr;
        return;
    }
}

void TFmicro_Network_Evaluate(float* inputs, float* outputs) {

    // Copy inputs to the interpreter's input tensor
    for (int k = 0; k < TF_MICRO_NUMBER_OF_INPUTS; k++) {
        interpreter->input(0)->data.f[k] = inputs[k];
    }

    interpreter->Invoke()

    // Copy the output tensor to the output array
    for (int k = 0; k < TF_MICRO_NUMBER_OF_OUTPUTS; k++) {
        outputs[k] = interpreter->output(0)->data.f[k];
    }
}

#endif
