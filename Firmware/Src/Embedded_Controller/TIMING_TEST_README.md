# Timing Test Module

This module provides platform-agnostic controller timing tests that work on both STM and Zynq platforms.

## Features

- **Platform Agnostic**: Works on both STM and Zynq using hardware bridge abstraction
- **Configurable**: Easy configuration through `timing_config.h`
- **Comprehensive**: Statistical analysis, frequency calculation, memory usage analysis
- **Clean Integration**: Single function call from main.c

## Usage

### 1. Enable Timing Test Mode

In `hardware_bridge.h`, uncomment the timing test flag:
```c
#define TIMING_TEST
```

### 2. Configure Controller

The timing test automatically uses the controller selected in `main.c` via the `select_controller()` function. No additional configuration needed!

To change the controller being tested, modify the `select_controller()` function in `main.c`:
```c
static const ControllerOps* select_controller(void)
{
    return &LQR_Ops;        // Test LQR controller
    // return &NeuralImitator_Ops;  // Test Neural Imitator
    // return &NNC_Ops;             // Test Neural Network Controller C
    // return &PID_Ops;             // Test PID controller
}
```

### 3. Configure Test Parameters

In `timing_config.h`, adjust test parameters:
```c
#define TIMING_ITERATIONS         1000    // Number of iterations
#define WARMUP_ITERATIONS         10      // Warmup runs
#define VERBOSE_OUTPUT            1       // Show individual timing values
#define TEST_MEMORY_USAGE         0       // Enable memory analysis
#define CONTINUOUS_TESTING        0       // Enable continuous testing
```

### 4. Compile and Run

The program will automatically:
- Detect the platform (STM or Zynq)
- Select the appropriate controller implementation
- Run comprehensive timing tests
- Display detailed statistics

## Platform Support

### STM Platform
- Uses `TIMER1_getSystemTime_Us()` for timing (via `GetTimeNow()`)
- Uses standard `printf()` for output
- Works with STM hardware bridge

### Zynq Platform  
- Uses `TIMER1_getSystemTime_Us()` for timing (via `GetTimeNow()`)
- Uses standard `printf()` for output
- Works with Zynq hardware bridge

### Controller Implementations
- **STM Platform**: Only `NeuralImitator_Ops` is available
- **Zynq Platform**: All controllers are available (`NeuralImitator_Ops`, `NNC_Ops`, `LQR_Ops`, `PID_Ops`, `PIDPos_Ops`)
- Platform-specific includes ensure only available controllers are compiled
- Automatic platform detection for display purposes

## Output

The timing test provides:
- **Mean execution time** (microseconds)
- **Standard deviation** (microseconds)
- **Min/Max execution times** (microseconds)
- **Frequency analysis** (Hz)
- **Throughput analysis** (iterations/sec)
- **Memory usage analysis** (if enabled)
- **Individual timing values** (if verbose mode enabled)

## Files

- `timing_test.h` - Header file with public interface
- `timing_test.c` - Platform-agnostic implementation
- `timing_config.h` - Configuration file
- `main.c` - Integration point (single function call)

## Example Output

```
=== Controller Timing Test Program ===
Version: 1.0 (Platform-Agnostic)
Platform: Zynq
Selected controller: 0x12345678

=== Configuration ===
Iterations: 1000
Warmup iterations: 10
Stats samples: 100
Verbose output: Yes
Memory test: No
Continuous testing: No

Warming up controller...
Running 1000 iterations...

=== Controller Evaluation Results ===
Iterations: 1000
Mean time: 45.23 us
Std deviation: 2.15 us
Min time: 42.10 us
Max time: 52.30 us
Frequency: 22107.45 Hz
Throughput: 22107.45 iterations/sec
```
