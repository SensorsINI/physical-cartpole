#include "angle_processing.h"
#include "parameters.h"
#include "median_filter.h"
#include <stdlib.h>
#include <string.h>
#include <limits.h> // For SHRT_MAX
#include "math.h"



// Averaging derivatives with median filter on Firmware
#if IROS_SHORT_POLE_PROFILE
/*
 * The current Development sensor/dead-zone pipeline balances the physical
 * short pole more reliably with this additional median filtering. The 2024
 * stack used size 1, so this is a current-hardware tuning, not legacy parity.
 */
#define ANGLE_D_BUFFER_SIZE 10
#define POSITION_D_BUFFER_SIZE 10
#else
#define ANGLE_D_BUFFER_SIZE 1 // Median filter for pole's angular velocity
#define POSITION_D_BUFFER_SIZE 1 // Median filter for cart's velocity
#endif

#define MAX_TIMESTEPS_FOR_DERIVATIVE 20


int angle_raw = 0, angle_raw_prev = -1;
float angle_raw_stable = -1;
float angleD_raw = 0, angleD_raw_stable = -1;
int freezme = 0;

// Hardware dead-zone detection (Zynq FPGA filter block). While the pole is in
// the potentiometer gap the ADC reads rail values and the filtered angle is
// garbage; the FPGA flags this per 2.2 us sample. hw_dz_valid stays 0 on STM.
static int hw_dz_valid = 0;
static int hw_dz_contaminated = 0;
static int hw_dz_dwell_polls = 0;
static int hw_dz_settling = 0;
static int hw_dz_settling_polls = 0;
// Longest continuous extrapolation allowed on hardware trigger. Swing-through
// and turnaround episodes measured on hardware last <200 ms; beyond this cap
// the pole is parked inside the gap and holding an extrapolated angle would
// just accumulate drift.
#define HW_DZ_MAX_EXTRAPOLATION_MS 500
// After the rail flag clears on a downward crossing (exit onto the high end of
// the track) the analog input still settles from the in-gap level (~8 counts)
// up to the true reading for ~15-20 ms; those samples are below the rail
// threshold but garbage (seen in CPP_mpc__2026-07-08_14-15-36: measured angle
// 0.79 rad off, derivative sign flip injected into MPC). So keep extrapolating
// after the flag clears until the measurement agrees with the extrapolated
// angle, with a time cap in case the extrapolation itself has drifted.
#define HW_DZ_SETTLING_TOLERANCE_ADC 100  // ~0.15 rad; > worst-case extrapolation drift over one episode
#define HW_DZ_SETTLING_MAX_MS 100

float angleDBuffer[ANGLE_D_BUFFER_SIZE]; // Buffer for angle derivatives, using int
float positionDBuffer[POSITION_D_BUFFER_SIZE]; // Buffer for position derivatives, also using int for processing

// Initialize buffer indices
unsigned short angleDBufferIndex = 0;
unsigned short positionDBufferIndex = 0;

// Helper function to update circular buffers for int values
void updateCircularBuffer(int* buffer, unsigned short* index, unsigned short size, int newValue) {
    buffer[*index] = newValue;
    *index = (*index + 1) % size; // Update index in a circular manner
}

void updateCircularBuffer_float(float* buffer, unsigned short* index, unsigned short size, float newValue) {
    buffer[*index] = newValue;
    *index = (*index + 1) % size; // Update index in a circular manner
}

// Function to average derivatives
void average_derivatives(float* angleDPtr, float* positionDPtr){
    // Update angleD buffer with current value
    updateCircularBuffer_float(angleDBuffer, &angleDBufferIndex, ANGLE_D_BUFFER_SIZE, *angleDPtr);

    updateCircularBuffer_float(positionDBuffer, &positionDBufferIndex, POSITION_D_BUFFER_SIZE, *positionDPtr);

    // Calculate medians using the updated buffers
    float angleDMedian = ClassicMedianFilter_float(angleDBuffer, ANGLE_D_BUFFER_SIZE); // Adjust casting if necessary
    float positionDMedian = ClassicMedianFilter_float(positionDBuffer, POSITION_D_BUFFER_SIZE); // Adjust casting if necessary

    // Update pointers with median values
    *angleDPtr = angleDMedian;
    *positionDPtr = positionDMedian; // Convert int back to short
}

void report_hardware_deadzone(int contaminated) {
	hw_dz_valid = 1;
	hw_dz_contaminated = contaminated;
}

void process_angle(int angleSamples[], unsigned short angleSampIndex, unsigned short angle_averageLen, int* anglePtr, int* angle_raw_Ptr, float* angleDPtr, int* invalid_stepPtr){
		int angle = ClassicMedianFilter(angleSamples, angle_averageLen);

		angle = wrapLocal(angle);
		*anglePtr = angle;
		*angle_raw_Ptr = angle;

		int invalid_step = anomaly_detection(angleSamples, angleSampIndex, angle_averageLen);
		// Surface hardware-detected dead-zone contamination in the invalid-step
		// counter (otherwise always 0 on Zynq, where angle_averageLen == 1), so
		// it shows up in PC-side recordings.
		if (hw_dz_valid && hw_dz_contaminated) {
			invalid_step++;
		}
		*invalid_stepPtr = invalid_step;

		treat_deadangle_with_derivative(anglePtr, invalid_step);

		*angleDPtr = angleD_raw;
}

// Anomaly Detection: count invalid buffer steps
int anomaly_detection(int* angleSamples, unsigned short angleSampIndex, unsigned short angle_averageLen){
	int invalid_step = 0;
	if(angle_averageLen > 1) {
		for (int i = 0; i < angle_averageLen; i++) {
			// start at oldest value (since angleSampIndex is not yet overwritten)
			int curr = angleSamples[(angleSampIndex + i) % angle_averageLen];
			int prev = angleSamples[(angleSampIndex + i + angle_averageLen - 1) % angle_averageLen];

			// previous value for oldest value not existing
			if(i != 0 && abs(wrapLocal(curr-prev)) > MAX_ADC_STEP)
				invalid_step++;
		}
	}
	return invalid_step;
}

int angle_history[MAX_TIMESTEPS_FOR_DERIVATIVE+1]; // Buffer to store past angles
int frozen_history[MAX_TIMESTEPS_FOR_DERIVATIVE+1]; // Buffer to store past angles
int idx_for_derivative_calculation_angle = 0; // Current index in the buffer
int angle_history_initialised = 0;

// Initialize the angle history buffer to -1
void init_angle_history() {

    for (int i = 0; i < MAX_TIMESTEPS_FOR_DERIVATIVE+1; ++i) {
        angle_history[i] = -1;
        frozen_history[i] = 0;
    }
}

float last_difference = 100000.0; //Just initialization value

void treat_deadangle_with_derivative(int* anglePtr, int invalid_step) {

	if (angle_history_initialised == 0)
	{
		init_angle_history();
		angle_history_initialised = 1;
	}


    // Calculate the index for the k-th past angle
    int kth_past_index = (idx_for_derivative_calculation_angle + 1) % (TIMESTEPS_FOR_DERIVATIVE+1);;
    int kth_past_angle = angle_history[kth_past_index];

    float current_difference = (kth_past_angle != -1) ? (float)(wrapLocal(*anglePtr - kth_past_angle)) / TIMESTEPS_FOR_DERIVATIVE : 0;

    if (last_difference > 10000) {
        last_difference = current_difference;
    }

    if (!IROS_SHORT_POLE_PROFILE && hw_dz_valid) {
        // Hardware-triggered handling: the FPGA filter block checks every
        // ~2.2 us XADC conversion against the rail thresholds, so this signal
        // is exact — no jump heuristic and no assumption about where the dead
        // zone sits. While contaminated, keep the freeze topped up so the
        // angle is extrapolated with the last stable derivative; when the
        // contamination ends, the freeze drains by itself: one more
        // extrapolated step, then measured angles with the derivative held
        // for TIMESTEPS_FOR_DERIVATIVE steps until the history is clean again.
        // This adapts to episode length (crossings ~30-80 ms, turnarounds up
        // to ~200 ms measured), where the old fixed 45/90 ms guess could
        // release mid-episode.
        if (hw_dz_contaminated) {
            hw_dz_dwell_polls++;
            hw_dz_settling = 1;
            hw_dz_settling_polls = 0;
        } else if (hw_dz_settling) {
            // Flag cleared but the analog reading may still be settling back
            // from the in-gap rail level: trust the measurement again only
            // once it re-converges to the extrapolated angle (or the cap
            // expires). *anglePtr is still this poll's measured angle here;
            // the freeze block below overwrites it while extrapolating.
            hw_dz_dwell_polls++;
            hw_dz_settling_polls++;
            float settling_residual = fabsf(wrapLocal_float((float)(*anglePtr) - angle_raw_stable));
            if (settling_residual <= HW_DZ_SETTLING_TOLERANCE_ADC ||
                hw_dz_settling_polls > (int)(HW_DZ_SETTLING_MAX_MS / POLLING_PERIOD_MS))
            {
                hw_dz_settling = 0;
                hw_dz_dwell_polls = 0;
            }
        } else {
            hw_dz_dwell_polls = 0;
        }
        if ((hw_dz_contaminated || hw_dz_settling) &&
            kth_past_angle != -1 &&
            hw_dz_dwell_polls <= (int)(HW_DZ_MAX_EXTRAPOLATION_MS / POLLING_PERIOD_MS) &&
            freezme < TIMESTEPS_FOR_DERIVATIVE + 3)
        {
            freezme = TIMESTEPS_FOR_DERIVATIVE + 3;
        }
    }
    // Heuristic fallback (STM, or Zynq firmware without the hardware flag):
    // detect the dead zone from a derivative jump near the known location.
    else if (kth_past_angle != -1 &&
        (angle_raw_stable > -500 && angle_raw_stable < 500) &&
        freezme == 0 &&
        (
		(TIMESTEPS_FOR_DERIVATIVE * abs(current_difference - last_difference) > POLLING_PERIOD_MS * 2.4)
		||
		(invalid_step > 5)
		)
		)
    {
        // Determine the freeze period based on the angle derivative stability
        if (angleD_raw_stable > 0) {
            freezme = (int)(45 / POLLING_PERIOD_MS) + TIMESTEPS_FOR_DERIVATIVE;  // Accelerate through the dead angle
        } else {
            freezme = (int)(90 / POLLING_PERIOD_MS) + TIMESTEPS_FOR_DERIVATIVE;  // Decelerate through the dead angle
        }
    }

    // Handle the frozen state
    if (freezme > 0) {
        --freezme;
        angleD_raw = angleD_raw_stable;
        if (freezme > TIMESTEPS_FOR_DERIVATIVE + 1) {
            angle_raw_stable += angleD_raw_stable;
            *anglePtr = (int)(angle_raw_stable);
        } else {
            angle_raw_stable = (float)(*anglePtr);
        }
    } else {
        angle_raw_stable = (float)(*anglePtr);
        angleD_raw = current_difference;
        angleD_raw_stable = angleD_raw;
    }

    last_difference = current_difference;

    // Save current angle in the history buffer and update index
    angle_history[idx_for_derivative_calculation_angle] = *anglePtr;
    frozen_history[idx_for_derivative_calculation_angle] = freezme;
    idx_for_derivative_calculation_angle = (idx_for_derivative_calculation_angle + 1) % (TIMESTEPS_FOR_DERIVATIVE+1); // Move to next index, wrap around if necessary
}

short position_history[MAX_TIMESTEPS_FOR_DERIVATIVE+1]; // Buffer to store past positions

int idx_for_derivative_calculation_position = 0; // Current index in the buffer
int position_history_initialised = 0;

// Initialize the angle history buffer to -1
void init_position_history() {

    for (int i = 0; i < MAX_TIMESTEPS_FOR_DERIVATIVE+1; ++i) {
        position_history[i] = SHRT_MAX;
        frozen_history[i] = 0;
    }
}


void calculate_position_difference_per_timestep(short* positionPtr, float* positionDPtr) {

	if (position_history_initialised == 0)
	{
		init_position_history();
		position_history_initialised = 1;
	}

    // Calculate the index for the k-th past angle
    int kth_past_index = (idx_for_derivative_calculation_position +1) % (TIMESTEPS_FOR_DERIVATIVE+1);
    int kth_past_position = position_history[kth_past_index];

    short not_normed_positionD_raw = kth_past_position != SHRT_MAX ? (*positionPtr - kth_past_position) :0;
    *positionDPtr =  (float)(not_normed_positionD_raw)/ TIMESTEPS_FOR_DERIVATIVE;
    position_history[idx_for_derivative_calculation_position] = *positionPtr;

    idx_for_derivative_calculation_position = (idx_for_derivative_calculation_position + 1) % (TIMESTEPS_FOR_DERIVATIVE+1); // Move to next index, wrap around if necessary
}


// Runtime override from the PC (CMD_SET_CONTROL_CONFIG). Restarts the history
// buffers because their circular indexing is modulo (TIMESTEPS_FOR_DERIVATIVE+1).
void set_timesteps_for_derivative(unsigned short timesteps) {
	if (timesteps < 1) {
		timesteps = 1;
	}
	if (timesteps > MAX_TIMESTEPS_FOR_DERIVATIVE) {
		timesteps = MAX_TIMESTEPS_FOR_DERIVATIVE;
	}
	TIMESTEPS_FOR_DERIVATIVE = timesteps;
	angle_history_initialised = 0;
	position_history_initialised = 0;
	idx_for_derivative_calculation_angle = 0;
	idx_for_derivative_calculation_position = 0;
	angleDBufferIndex = 0;
	positionDBufferIndex = 0;
	memset(angleDBuffer, 0, sizeof(angleDBuffer));
	memset(positionDBuffer, 0, sizeof(positionDBuffer));
	last_difference = 100000.0;
	freezme = 0;
}


int wrapLocal(int angle) {
    if (angle > ANGLE_360_DEG_IN_ADC_UNITS/2)
		return angle - ANGLE_360_DEG_IN_ADC_UNITS;
	if (angle <= -ANGLE_360_DEG_IN_ADC_UNITS/2)
		return angle + ANGLE_360_DEG_IN_ADC_UNITS;
	else
		return angle;
}


float wrapLocal_float(float angle) {
    if (angle > ANGLE_360_DEG_IN_ADC_UNITS/2)
		return angle - ANGLE_360_DEG_IN_ADC_UNITS;
	if (angle <= -ANGLE_360_DEG_IN_ADC_UNITS/2)
		return angle + ANGLE_360_DEG_IN_ADC_UNITS;
	else
		return angle;
}


float wrapLocal_rad(float angle) {
    if (angle > M_PI)
		return angle - 2*M_PI;
	if (angle <= -M_PI)
		return angle + 2*M_PI;
	else
		return angle;
}


int unwrapLocal(int previous, int current) {
	int diff = current-previous;

	if (diff > ANGLE_360_DEG_IN_ADC_UNITS/2)
		return current - ANGLE_360_DEG_IN_ADC_UNITS;
	if (diff < -ANGLE_360_DEG_IN_ADC_UNITS/2)
		return current + ANGLE_360_DEG_IN_ADC_UNITS;
	else
		return current;
}

int wrap(int current) {
	if(current > 0)
		return current - ANGLE_360_DEG_IN_ADC_UNITS * (current / ANGLE_360_DEG_IN_ADC_UNITS);
	else
		return current + ANGLE_360_DEG_IN_ADC_UNITS * (current / ANGLE_360_DEG_IN_ADC_UNITS + 1);
}

int unwrap(int previous, int current) {
    int diff = previous-current;
	if (diff>0)
    	return current + ANGLE_360_DEG_IN_ADC_UNITS * (((2 * diff) / ANGLE_360_DEG_IN_ADC_UNITS + 1) / 2);
	else
    	return current + ANGLE_360_DEG_IN_ADC_UNITS * (((2 * diff) / ANGLE_360_DEG_IN_ADC_UNITS - 1) / 2);
}
