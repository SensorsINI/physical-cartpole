/*
 * Platform-Agnostic Timing Test Module Implementation
 * 
 * This module contains timing test functionality that works on both STM and Zynq
 * platforms using the hardware bridge abstraction.
 */

#include "timing_test.h"


#include "timing_config.h"
#include "controller_manager.h"
#include "hardware_bridge.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "neural_controller_C.h"
#include "lqr.h"
#include "hardware_pid.h"

#ifdef TIMING_TEST

/* Include platform-specific controller implementations */
#ifdef STM
#include "STM/neural_imitator.h"
#elif defined(ZYNQ)
#include "Zynq/neural_imitator.h"
#else
#error "Platform not supported. Define either STM or ZYNQ"
#endif

/******************** Helper Functions **********************************/

/* Generate realistic test inputs for the controller */
static void generate_test_inputs(float* inputs, int num_inputs)
{
    static float time_counter = 0.0f;
    time_counter += TEST_TIME_STEP;
    
    for (int i = 0; i < num_inputs; i++) {
        switch (i) {
            case 0:  // angle
                inputs[i] = TEST_ANGLE_AMPLITUDE * sinf(time_counter * 2.0f * M_PI * TEST_ANGLE_FREQUENCY);
                break;
            case 1:  // angleD
                inputs[i] = TEST_ANGLE_AMPLITUDE * 2.0f * M_PI * TEST_ANGLE_FREQUENCY * 
                           cosf(time_counter * 2.0f * M_PI * TEST_ANGLE_FREQUENCY);
                break;
            case 2:  // position
                inputs[i] = TEST_POSITION_AMPLITUDE * sinf(time_counter * 2.0f * M_PI * TEST_POSITION_FREQUENCY);
                break;
            case 3:  // positionD
                inputs[i] = TEST_POSITION_AMPLITUDE * 2.0f * M_PI * TEST_POSITION_FREQUENCY * 
                           cosf(time_counter * 2.0f * M_PI * TEST_POSITION_FREQUENCY);
                break;
            case 4:  // target_position
                inputs[i] = 0.0f;
                break;
            case 5:  // time
                inputs[i] = time_counter;
                break;
            default:
                inputs[i] = 0.0f;
                break;
        }
    }
}

/* Calculate statistics from timing data */
static void calculate_statistics(unsigned long* times, int count, float* mean, float* std_dev, 
                                unsigned long* min_time, unsigned long* max_time)
{
    if (count == 0) {
        *mean = 0.0f;
        *std_dev = 0.0f;
        *min_time = 0;
        *max_time = 0;
        return;
    }
    
    unsigned long total = 0;
    *min_time = times[0];
    *max_time = times[0];
    
    for (int i = 0; i < count; i++) {
        total += times[i];
        if (times[i] < *min_time) *min_time = times[i];
        if (times[i] > *max_time) *max_time = times[i];
    }
    
    *mean = (float)total / count;
    
    float variance = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = (float)times[i] - *mean;
        variance += diff * diff;
    }
    variance /= count;
    *std_dev = sqrtf(variance);
    
    // Debug: Print raw values to check for corruption
    printf("DEBUG: calculate_statistics - count=%d, total=%lu, mean=%.6f\r\n", 
           count, total, *mean);
}

/* Convert microseconds to float for display */
static float us_to_float(unsigned long us)
{
    return (float)us;
}

/* Convert clock cycles to nanoseconds */
static float cycles_to_ns(unsigned long cycles)
{
    // Clock frequency is 333.333 MHz (from CLOCK_FREQ in hardware_bridge.h)
    return (float)cycles * 1000.0f / 333.333f; // Convert to nanoseconds
}


/* Print timing results using standard printf (supports float formatting) */
static void print_timing_results(const char* test_name, unsigned long* times, int count, 
                                float mean_cycles, float std_dev_cycles, unsigned long min_cycles, unsigned long max_cycles)
{
    // Convert clock cycles to microseconds and nanoseconds
    float mean_us = cycles_to_ns((unsigned long)mean_cycles) / 1000.0f;
    float std_dev_us = cycles_to_ns((unsigned long)std_dev_cycles) / 1000.0f;
    float min_us = cycles_to_ns(min_cycles) / 1000.0f;
    float max_us = cycles_to_ns(max_cycles) / 1000.0f;
    
    printf("\r\n=== %s Results ===\r\n", test_name);
    printf("Iterations: %d\r\n", count);
    printf("Mean time: %.2f us (%.0f ns, %.0f cycles)\r\n", mean_us, mean_us * 1000.0f, mean_cycles);
    printf("Std deviation: %.2f us (%.0f ns, %.0f cycles)\r\n", std_dev_us, std_dev_us * 1000.0f, std_dev_cycles);
    printf("Min time: %.2f us (%.0f ns, %lu cycles)\r\n", min_us, min_us * 1000.0f, min_cycles);
    printf("Max time: %.2f us (%.0f ns, %lu cycles)\r\n", max_us, max_us * 1000.0f, max_cycles);
    
#if SHOW_FREQUENCY_ANALYSIS
    if (mean_us > 0) {
        printf("Frequency: %.2f Hz\r\n", 1000000.0f / mean_us);
    } else {
        printf("Frequency: N/A (too fast to measure)\r\n");
    }
#endif

#if SHOW_THROUGHPUT_ANALYSIS
    if (mean_us > 0) {
        printf("Throughput: %.2f iterations/sec\r\n", 1000000.0f / mean_us);
    } else {
        printf("Throughput: N/A (too fast to measure)\r\n");
    }
#endif
    
    if (VERBOSE_OUTPUT && count <= 20) {
        printf("Individual times (us): ");
        for (int i = 0; i < count; i++) {
            printf("%.2f ", us_to_float(times[i]));
        }
        printf("\r\n");
    }
}

/* Print controller information */
static void print_controller_info(const ControllerSpec* spec)
{
#if SHOW_CONTROLLER_INFO
    printf("\r\n=== Controller Information ===\r\n");
    printf("Version: %d\r\n", spec->version);
    printf("Inputs: %d, Outputs: %d\r\n", spec->n_inputs, spec->n_outputs);
    
    if (spec->names) {
        printf("Input names: ");
        for (int i = 0; i < spec->n_inputs; i++) {
            printf("%s ", spec->names[i]);
        }
        printf("\r\n");
    }
#endif
}

/* Get the currently active controller from controller manager */
static const ControllerOps* get_active_controller(void)
{
    return CR_GetActive();
}

/* Run controller timing test */
static void run_controller_timing_test(void)
{
    const ControllerOps* controller = get_active_controller();
    if (!controller || !controller->spec) {
        printf("ERROR: No active controller or invalid spec\r\n");
        return;
    }
    
    const ControllerSpec* spec = controller->spec();
    int num_inputs = spec->n_inputs;
    int num_outputs = spec->n_outputs;
    
    print_controller_info(spec);
    
    // Allocate input/output arrays
    float inputs[MAX_INPUTS];
    float outputs[MAX_OUTPUTS];
    
    // Warmup runs
    printf("Warming up controller...\r\n");
    for (int i = 0; i < WARMUP_ITERATIONS; i++) {
        generate_test_inputs(inputs, num_inputs);
        if (controller->evaluate) {
            controller->evaluate(inputs, outputs);
        }
    }
    
    // Main timing test - use stack allocation instead of heap
    unsigned long iteration_times[TIMING_ITERATIONS];
    
    printf("Running %d iterations...\r\n", TIMING_ITERATIONS);
    
    // Since controller is very fast, measure multiple iterations at once for better accuracy
    const int iterations_per_measurement = 10;
    int measurement_count = TIMING_ITERATIONS / iterations_per_measurement;
    
    for (int i = 0; i < measurement_count; i++) {
        unsigned long iter_start, iter_end;
        iter_start = GetTimeNowHighRes();
        
        // Run multiple iterations
        for (int j = 0; j < iterations_per_measurement; j++) {
            generate_test_inputs(inputs, num_inputs);
            if (controller->evaluate) {
                controller->evaluate(inputs, outputs);
            }
        }
        
        iter_end = GetTimeNowHighRes();
        iteration_times[i] = (iter_end - iter_start) / iterations_per_measurement;
    }
    
    // Fill remaining slots with the last measurement
    for (int i = measurement_count; i < TIMING_ITERATIONS; i++) {
        iteration_times[i] = iteration_times[measurement_count - 1];
    }
    
    // Filter out invalid timing measurements (very large values likely due to timer overflow)
    int valid_count = 0;
    unsigned long filtered_times[TIMING_ITERATIONS];
    for (int i = 0; i < TIMING_ITERATIONS; i++) {
        // Filter out values that are too large (likely timer overflow) or negative
        if (iteration_times[i] < 1000000) { // Less than 1 second
            filtered_times[valid_count] = iteration_times[i];
            valid_count++;
        }
    }
    
    // Debug: Print first few raw and filtered values
    printf("DEBUG: First 5 raw values (cycles): ");
    for (int i = 0; i < 5 && i < TIMING_ITERATIONS; i++) {
        printf("%lu ", iteration_times[i]);
    }
    printf("\r\n");
    
    printf("DEBUG: First 5 filtered values (cycles): ");
    for (int i = 0; i < 5 && i < valid_count; i++) {
        printf("%lu ", filtered_times[i]);
    }
    printf("\r\n");
    
    if (valid_count == 0) {
        printf("WARNING: All timing measurements were invalid (timer overflow)\r\n");
        return;
    }
    
    if (valid_count < TIMING_ITERATIONS) {
        printf("WARNING: %d out of %d timing measurements were invalid (filtered out)\r\n", 
               TIMING_ITERATIONS - valid_count, TIMING_ITERATIONS);
    }
    
    // Calculate statistics on filtered data (in clock cycles)
    float mean_cycles, std_dev_cycles;
    unsigned long min_cycles, max_cycles;
    calculate_statistics(filtered_times, valid_count, 
                        &mean_cycles, &std_dev_cycles, &min_cycles, &max_cycles);
    
    // Debug: Print what we got
    printf("DEBUG: valid_count=%d, mean=%.2f cycles, std=%.2f cycles, min=%lu cycles, max=%lu cycles\r\n", 
           valid_count, mean_cycles, std_dev_cycles, min_cycles, max_cycles);
    
    // Debug: Test simple printf
    printf("DEBUG: Simple test - 1.23 = %.2f\r\n", 1.23f);
    
    // Print results (pass clock cycles, let print function handle conversion)
    print_timing_results("Controller Evaluation", filtered_times, valid_count,
                        mean_cycles, std_dev_cycles, min_cycles, max_cycles);
    
    // Memory usage test (if enabled)
#if TEST_MEMORY_USAGE && SHOW_MEMORY_ANALYSIS
    printf("\r\n=== Memory Usage Analysis ===\r\n");
    printf("Input array size: %d bytes\r\n", num_inputs * sizeof(float));
    printf("Output array size: %d bytes\r\n", num_outputs * sizeof(float));
    printf("Timing data size: %d bytes\r\n", TIMING_ITERATIONS * sizeof(unsigned long));
    printf("Total working memory: %d bytes\r\n", 
           (num_inputs + num_outputs) * sizeof(float) + TIMING_ITERATIONS * sizeof(unsigned long));
#endif
    
    // Cleanup - no longer needed with stack allocation
}

/* Run detailed statistics on a subset of iterations */
static void run_detailed_statistics(void)
{
    const ControllerOps* controller = get_active_controller();
    if (!controller || !controller->evaluate) {
        return;
    }
    
    const ControllerSpec* spec = controller->spec();
    int num_inputs = spec->n_inputs;
    
    float inputs[MAX_INPUTS];
    float outputs[MAX_OUTPUTS];
    unsigned long times[STATS_SAMPLES];
    
    printf("\r\nRunning detailed statistics on %d samples...\r\n", STATS_SAMPLES);
    
    // Use same improved timing approach as main test
    const int iterations_per_measurement = 10;
    int measurement_count = STATS_SAMPLES / iterations_per_measurement;
    
    for (int i = 0; i < measurement_count; i++) {
        unsigned long start, end;
        start = GetTimeNow();
        
        // Run multiple iterations
        for (int j = 0; j < iterations_per_measurement; j++) {
            generate_test_inputs(inputs, num_inputs);
            if (controller->evaluate) {
                controller->evaluate(inputs, outputs);
            }
        }
        
        end = GetTimeNow();
        times[i] = (end - start) / iterations_per_measurement;
    }
    
    // Fill remaining slots with the last measurement
    for (int i = measurement_count; i < STATS_SAMPLES; i++) {
        times[i] = times[measurement_count - 1];
    }
    
    // Calculate statistics on filtered data (in clock cycles)
    float mean_cycles, std_dev_cycles;
    unsigned long min_cycles, max_cycles;
    calculate_statistics(times, STATS_SAMPLES, 
                        &mean_cycles, &std_dev_cycles, &min_cycles, &max_cycles);
    
    print_timing_results("Detailed Statistics", times, STATS_SAMPLES,
                        mean_cycles, std_dev_cycles, min_cycles, max_cycles);
    
}

/******************** Public Interface **********************************/

/* Main entry point for timing tests */
void run_timing_test_suite(void)
{
    printf("\r\n=== Controller Timing Test Program ===\r\n");
    printf("Version: 1.0 (Platform-Agnostic)\r\n");
    printf("Build: %s %s\r\n", __DATE__, __TIME__);
    
#ifdef STM
    printf("Platform: STM\r\n");
#elif defined(ZYNQ)
    printf("Platform: Zynq\r\n");
#endif
    
    // Select controller for timing test
    const ControllerOps* controller = get_active_controller();
    if (!controller) {
        printf("ERROR: No controller selected\r\n");
        return;
    }
    
    printf("Selected controller: %p\r\n", controller);
    
    // Initialize controller if needed
    if (controller->init) {
        printf("Initializing controller...\r\n");
        controller->init();
    }
    
    // Print configuration
    printf("\r\n=== Configuration ===\r\n");
    printf("Iterations: %d\r\n", TIMING_ITERATIONS);
    printf("Warmup iterations: %d\r\n", WARMUP_ITERATIONS);
    printf("Stats samples: %d\r\n", STATS_SAMPLES);
    printf("Verbose output: %s\r\n", VERBOSE_OUTPUT ? "Yes" : "No");
    printf("Memory test: %s\r\n", TEST_MEMORY_USAGE ? "Yes" : "No");
    printf("Continuous testing: %s\r\n", CONTINUOUS_TESTING ? "Yes" : "No");
    
    // Run timing tests
    run_controller_timing_test();
    run_detailed_statistics();
    
    // Cleanup
    if (controller->release) {
        printf("\r\nReleasing controller...\r\n");
        controller->release();
    }
    
    printf("\r\n=== Timing Test Complete ===\r\n");
    
    // Continuous testing mode
#if CONTINUOUS_TESTING
    printf("Entering continuous testing mode...\r\n");
    while(1) {
        run_controller_timing_test();
        Sleep_ms(CONTINUOUS_TEST_DELAY);
    }
#else
    // Keep running for single test
    while(1) {
        // Idle loop
    }
#endif
}

#else /* TIMING_TEST not defined */

/* Stub implementation when timing test is disabled */
void run_timing_test_suite(void)
{
    /* Do nothing when timing test is disabled */
}

#endif /* TIMING_TEST */
