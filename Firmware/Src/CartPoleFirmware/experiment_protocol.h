#ifndef __EXPERIMENT_PROTOCOL_H_
#define __EXPERIMENT_PROTOCOL_H_

void HardwareExperimentProtocol(
		float position, float angle, float time,
		float* target_position, float* target_equilibrium,
		int* run_hardware_experiment, int* save_to_offline_buffers,
		bool* ControlOnChip_Enabled_var, int* motor_command, bool* USE_TARGET_SWITCHES_var);

#endif /*__EXPERIMENT_PROTOCOL_H_*/
