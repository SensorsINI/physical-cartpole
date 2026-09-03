#ifndef __COMMUNICATION_WITH_PC_H_
#define __COMMUNICATION_WITH_PC_H_

#include "communication_with_PC_general.h"
#include <stdbool.h>

// State packet length includes an 8-byte accumulated chip timestamp, 1 SecLoc telemetry byte
// (bit 0 = skipped_update, bit 1 = gate_skipped, bit 2 = step computed by the PL backend,
// bit 3 = PL fault: PL backend selected but the PL block is absent or the transaction
// failed; the step output zero force, no SW fallback). Bits 4..7 carry the
// angle-calibration revision. The packet also carries target_equilibrium (float).
#define STATE_MESSAGE_LEN 40


int get_command_from_PC_message(unsigned char * rxBuffer, unsigned int* rxCnt);
void prepare_message_to_PC_state(
		unsigned char * buffer,
		unsigned short message_len,
		int angle,
		float angleD_unprocessed,
		short position,
		float target_position,
		float target_equilibrium,
		int motor_command,
		int invalid_step,
		unsigned long time_difference_between_measurement,
		unsigned long long timeMeasured,
		unsigned long latency,
		unsigned short	latency_violation,
		unsigned char secloc_flags
		);

/* Reply to CMD_GET_SECLOC_INFO: SecLoc execution backend diagnostics.
 * backend: 0 = SW, 1 = PL, 2 = PL shadow. This is the requested backend and
 * is never silently degraded; if it is PL-type while pl_available = 0 every
 * step outputs zero force and is counted in pl_fault_count. */
void prepare_message_to_PC_secloc_info(
		unsigned char * buffer,
		unsigned char backend,
		unsigned char pl_available,
		unsigned int shadow_mismatch_count,
		unsigned int pl_update_count,
		unsigned int pl_nn_wait_cycles,
		unsigned int pl_fault_count
		);

void prepare_message_to_PC_calibration(unsigned char * buffer, int encoderDirection);
void send_information_experiment_done(unsigned char * buffer, unsigned short experiment_length);

void prepare_message_to_PC_control_config(
		unsigned char * txBuffer,
		unsigned short polling_period,
		bool controlSync,
		float angle_hanging,
		unsigned short angle_averageLen,
		bool correct_motor_dynamics,
		unsigned short timesteps_for_derivative,
		unsigned char hanging_status
		);

void prepare_message_to_PC_angle_calibration(
		unsigned char *txBuffer,
		float angle_circle,
		unsigned char calibration_status
		);

void prepare_buffer_to_send_long(unsigned char* Buffer, unsigned char CMD, unsigned int message_length);

#endif /*__COMMUNICATION_WITH_PC_H_*/
