#include "angle_processing.h"
#include "parameters.h"
#include "median_filter.h"
#include <stdlib.h>
#include "math.h"



// Averaging derivatives with median filter
#define ANGLE_D_BUFFER_SIZE 3 // Median filter for pole's angular velocity
#define POSITION_D_BUFFER_SIZE 3 // Median filter for cart's velocity
#define timesteps_for_derivatives 6 // Based on how many timesteps should the derivative be calculated



const int ADC_RANGE = 65536;

int angle_raw = 0, angle_raw_prev = -1, angle_raw_stable = -1, angle_raw_sensor;
int angleD_raw = 0, angleD_raw_stable = -1, angleD_raw_sensor;
int frozen = 0;

int angleDBuffer[ANGLE_D_BUFFER_SIZE]; // Buffer for angle derivatives, using int
int positionDBuffer[POSITION_D_BUFFER_SIZE]; // Buffer for position derivatives, also using int for processing

// Initialize buffer indices
unsigned short angleDBufferIndex = 0;
unsigned short positionDBufferIndex = 0;

// Helper function to update circular buffers for int values
void updateCircularBuffer(int* buffer, unsigned short* index, unsigned short size, int newValue) {
    buffer[*index] = newValue;
    *index = (*index + 1) % size; // Update index in a circular manner
}

// Function to average derivatives
void average_derivatives(int* angleDPtr, short* positionDPtr){
    // Update angleD buffer with current value
    updateCircularBuffer(angleDBuffer, &angleDBufferIndex, ANGLE_D_BUFFER_SIZE, *angleDPtr);

    // Convert short value to int and update positionD buffer
    int positionDInt = (int)*positionDPtr; // Convert short to int for processing
    updateCircularBuffer(positionDBuffer, &positionDBufferIndex, POSITION_D_BUFFER_SIZE, positionDInt);

    // Calculate medians using the updated buffers
    int angleDMedian = ClassicMedianFilter(angleDBuffer, ANGLE_D_BUFFER_SIZE); // Adjust casting if necessary
    int positionDMedian = ClassicMedianFilter((int*)positionDBuffer, POSITION_D_BUFFER_SIZE); // Adjust casting if necessary

    // Update pointers with median values
    *angleDPtr = angleDMedian;
    *positionDPtr = (short)positionDMedian; // Convert int back to short
}

void process_angle(int angleSamples[], unsigned short angleSampIndex, unsigned short angle_averageLen, int* anglePtr, int* angleDPtr, int* invalid_stepPtr){
		int angle = ClassicMedianFilter(angleSamples, angle_averageLen);
		*anglePtr = angle;

		int invalid_step = anomaly_detection(angleSamples, angleSampIndex, angle_averageLen);
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



#define BUFFER_SIZE (timesteps_for_derivatives+1)
int angle_history[BUFFER_SIZE]; // Buffer to store past angles
int frozen_history[BUFFER_SIZE]; // Buffer to store past angles
int angleIndex = 0; // Current index in the buffer
int angle_history_initialised = 0;

// Initialize the angle history buffer to -1
void init_angle_history() {
    for (int i = 0; i < BUFFER_SIZE; ++i) {
        angle_history[i] = -1;
        frozen_history[i] = 0;
    }
}


void treat_deadangle_with_derivative(int* anglePtr, int invalid_step) {

	if (angle_history_initialised == 0)
	{
		init_angle_history();
		angle_history_initialised = 1;
	}


    // Calculate the index for the k-th past angle
    int kth_past_index = (angleIndex - timesteps_for_derivatives + BUFFER_SIZE) % BUFFER_SIZE;
    int kth_past_angle = angle_history[kth_past_index];
    int kth_past_frozen = frozen_history[kth_past_index];

    // Anomaly Detection: unstable buffer (invalid steps) or unstable angle_raw (jump in angle_raw), only inside region close to 0
    if (kth_past_angle != -1 &&
       ((invalid_step > 5 && abs(wrapLocal(kth_past_angle)) < ADC_RANGE/20) ||
       (abs(wrapLocal(*anglePtr - kth_past_angle)) > ADC_RANGE/8 && kth_past_frozen < 3))) {

        frozen++;
        *anglePtr = angle_raw_stable != -1 ? angle_raw_stable : 0;
        angleD_raw = angleD_raw_stable != -1 ? angleD_raw_stable : 0;
    } else {
        angleD_raw = angle_raw_stable != -1 ? wrapLocal(*anglePtr - kth_past_angle) / ((timesteps_for_derivatives-1) + kth_past_frozen + 1) : 0;
        angle_raw_stable = *anglePtr;
        angleD_raw_stable = angleD_raw;
        frozen = 0;
    }

    angle_raw_sensor = *anglePtr;
    angleD_raw_sensor = angleD_raw;

    // Save previous values
    angle_raw_prev = *anglePtr;

    // Save current angle in the history buffer and update index
    angle_history[angleIndex] = *anglePtr;
    frozen_history[angleIndex] = frozen;
    angleIndex = (angleIndex + 1) % BUFFER_SIZE; // Move to next index, wrap around if necessary
}


int wrapLocal(int angle) {
    if (angle > ADC_RANGE/2)
		return angle - ADC_RANGE;
	if (angle <= -ADC_RANGE/2)
		return angle + ADC_RANGE;
	else
		return angle;
}


float wrapLocal_float(float angle) {
    if (angle > ADC_RANGE/2)
		return angle - ADC_RANGE;
	if (angle <= -ADC_RANGE/2)
		return angle + ADC_RANGE;
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

	if (diff > ADC_RANGE/2)
		return current - ADC_RANGE;
	if (diff < -ADC_RANGE/2)
		return current + ADC_RANGE;
	else
		return current;
}

int wrap(int current) {
	if(current > 0)
		return current - ADC_RANGE * (current / ADC_RANGE);
	else
		return current + ADC_RANGE * (current / ADC_RANGE + 1);
}

int unwrap(int previous, int current) {
    int diff = previous-current;
	if (diff>0)
    	return current + ADC_RANGE * (((2 * diff) / ADC_RANGE + 1) / 2);
	else
    	return current + ADC_RANGE * (((2 * diff) / ADC_RANGE - 1) / 2);
}
