#include "angle_processing.h"
#include "parameters.h"
#include "median_filter.h"
#include <stdlib.h>
#include "math.h"

int angle_raw = 0, angle_raw_prev = -1, angle_raw_stable = -1, angle_raw_sensor;
int angleD_raw = 0, angleD_raw_stable = -1, angleD_raw_sensor;
int frozen = 0;


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



void treat_deadangle_with_derivative(int* anglePtr, int invalid_step) {
    // Anomaly Detection: unstable buffer (invalid steps) or unstable angle_raw (jump in angle_raw), only inside region close to 0
    if (angle_raw_prev != -1 &&
       ((invalid_step > 5 && abs(wrapLocal(angle_raw_prev)) < 200) ||
       (abs(wrapLocal(*anglePtr - angle_raw_prev)) > 500 && frozen < 3))) {

        frozen++;
        *anglePtr = angle_raw_stable != -1 ? angle_raw_stable : 0;
        angleD_raw = angleD_raw_stable != -1 ? angleD_raw_stable : 0;
    } else {
        angleD_raw = angle_raw_stable != -1 ? wrapLocal(*anglePtr - angle_raw_stable) / (frozen + 1) : 0;
        angle_raw_stable = *anglePtr;
        angleD_raw_stable = angleD_raw;
        frozen = 0;
    }

    angle_raw_sensor = *anglePtr;
    angleD_raw_sensor = angleD_raw;

    // Save previous values
    angle_raw_prev = *anglePtr;
}


const int ADC_RANGE = 4096;

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

	if (diff > 2000)
		return current - ADC_RANGE;
	if (diff < -2000)
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
