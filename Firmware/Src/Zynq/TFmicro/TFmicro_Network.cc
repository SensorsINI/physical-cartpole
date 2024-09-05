#include "TFmicro_Network.h"

#include "TFmicroZynqLib/tfmicrozynq.h"
#include <stdint.h>

extern const unsigned char model_array[];

// Anonymous namespace to restrict visibility to this file
namespace {
  // Local typedef for the operator resolver (keeping the 'tflite' usage here for internal logic)
  using TFmicroOpResolver = tflite::MicroMutableOpResolver<2>;

  // Internal state variables
  tflite::MicroInterpreter* interpreter = nullptr;  // Keep 'interpreter' internal to this file
  constexpr int kTensorArenaSize = TF_MICRO_ARENA_SIZE;
  uint8_t tensor_arena[kTensorArenaSize];

  // Function to register operations locally
  TfLiteStatus RegisterOps(TFmicroOpResolver& op_resolver) {
    TF_LITE_ENSURE_STATUS(op_resolver.AddFullyConnected());
    TF_LITE_ENSURE_STATUS(op_resolver.AddTanh());  // Register TANH operation
    return kTfLiteOk;
  }
}

// Now define the global functions declared in the header file without namespaces
void TFmicro_Network_Init() {
  // Use the tflite internal methods and types, but no longer need the 'tflite::' prefix in global scope
  const tflite::Model* model = ::tflite::GetModel(model_array);

  static TFmicroOpResolver op_resolver;  // Keep op_resolver local
  RegisterOps(op_resolver);

  // Create interpreter with local arena and variables
  interpreter = new tflite::MicroInterpreter(model, op_resolver, tensor_arena, kTensorArenaSize);
  interpreter->AllocateTensors();
}

void TFmicro_Network_Evaluate(float* inputs, float* outputs) {
  const int num_inputs = TF_MICRO_NUMBER_OF_INPUTS;
  const int num_outputs = TF_MICRO_NUMBER_OF_OUTPUTS;

  // Copy input data to the interpreter's input tensor
  for (int i = 0; i < num_inputs; i++) {
    interpreter->input(0)->data.f[i] = inputs[i];
  }

  // Run inference
  interpreter->Invoke();

  // Copy the result from the interpreter's output tensor to the output buffer
  for (int i = 0; i < num_outputs; i++) {
    outputs[i] = interpreter->output(0)->data.f[i];
  }
}



