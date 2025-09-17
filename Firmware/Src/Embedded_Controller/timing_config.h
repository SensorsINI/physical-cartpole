/*
 * Timing Configuration Header
 * 
 * This header file allows easy configuration of the timing test program
 * without modifying the main source code.
 */

#ifndef TIMING_CONFIG_H
#define TIMING_CONFIG_H

/******************** Controller Selection **********************************/

/* Controller is selected in main.c via select_controller() function */
/* No need to configure controller here - timing test uses the active controller */

/******************** Timing Configuration **********************************/

/* Number of iterations for main timing test */
#define TIMING_ITERATIONS         1000

/* Number of warmup iterations before timing starts */
#define WARMUP_ITERATIONS         1

/* Maximum number of iterations (safety limit) */
#define MAX_ITERATIONS            10000

/* Enable verbose output (shows individual timing values) */
#define VERBOSE_OUTPUT            1

/* Enable memory usage testing */
#define TEST_MEMORY_USAGE         1

/* Enable continuous testing mode */
#define CONTINUOUS_TESTING        1

/* Delay between continuous tests (ms) */
#define CONTINUOUS_TEST_DELAY     1000

/******************** Test Input Configuration **********************************/

/* Test input parameters */
#define TEST_ANGLE_AMPLITUDE      0.1f
#define TEST_ANGLE_FREQUENCY      0.5f
#define TEST_POSITION_AMPLITUDE   0.05f
#define TEST_POSITION_FREQUENCY   0.3f
#define TEST_TIME_STEP            0.001f

/******************** Output Configuration **********************************/

/* Enable detailed controller information */
#define SHOW_CONTROLLER_INFO      1

/* Enable frequency analysis */
#define SHOW_FREQUENCY_ANALYSIS   1

/* Enable throughput analysis */
#define SHOW_THROUGHPUT_ANALYSIS  1

/* Enable memory analysis */
#define SHOW_MEMORY_ANALYSIS      1

#endif /* TIMING_CONFIG_H */
