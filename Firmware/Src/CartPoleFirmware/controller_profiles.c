#include "controller_profiles.h"
#include "parameters.h"
#include "angle_processing.h"
#include "hardware_bridge.h"
#include "secloc_controller.h"

#ifdef ZYNQ
#include "rpgd_zynq_30ms_config.h"
#endif

#define MOTOR_MAP_SHOW 0.5733488f, 0.0257380f, 0.0258429f
#define SLIDER_TARGET_HALF_LENGTH_SHOW 0.12f

typedef struct {
	unsigned short period_ms;
	unsigned short derivative_n;
	int pwm_period;
	float motor_correction[3];
	float encoder_range;
	unsigned short derivative_median_len;
	int fpga_dz_extrapolation;
} ShowProfile;

static const ShowProfile kProfileLong = {
	.period_ms = 10,
	.derivative_n = 2,
	.pwm_period = 10000,
	.motor_correction = {MOTOR_MAP_SHOW},
	.encoder_range = 4695.0f,
	.derivative_median_len = 1,
	.fpga_dz_extrapolation = 1,
};

static const ShowProfile kProfileRpgd = {
#ifdef RPGD_CONTROL_PERIOD_MS
	.period_ms = (unsigned short)RPGD_CONTROL_PERIOD_MS,
#else
	.period_ms = 20,
#endif
#ifdef RPGD_30MS_DERIVATIVE_STEPS
	.derivative_n = (unsigned short)RPGD_30MS_DERIVATIVE_STEPS,
#else
	.derivative_n = 1,
#endif
	.pwm_period = 10000,
	.motor_correction = {MOTOR_MAP_SHOW},
	.encoder_range = 4695.0f,
	.derivative_median_len = 1,
	.fpga_dz_extrapolation = 1,
};

static const ShowProfile kProfileShortPl = {
	.period_ms = 1,
	.derivative_n = 10,
	.pwm_period = 2500,
	.motor_correction = {MOTOR_MAP_SHOW},
	.encoder_range = 4705.0f,
	.derivative_median_len = 10,
	.fpga_dz_extrapolation = 0,
};

bool show_switch_mux_enabled(void)
{
#if defined(ZYNQ) && defined(ZYBO_Z720)
	return true;
#else
	return false;
#endif
}

int show_mux_controller_from_switches(unsigned int switch_bits)
{
	unsigned int bits = switch_bits & SHOW_SWITCH_MUX_MASK;
	if (bits == 0u || (bits & (bits - 1u)) != 0u) {
		return SHOW_MUX_IDLE;
	}
	if (bits & 0x1u) {
		return OnChipController_RPGD;
	}
	if (bits & 0x2u) {
		return OnChipController_neural_controller_C;
	}
	if (bits & 0x4u) {
		return OnChipController_neural_controller_LSTM_C;
	}
	return OnChipController_NeuralImitator;
}

int show_mux_debounced_controller(unsigned int switch_bits)
{
	static int stable = SHOW_MUX_UNSTABLE;
	static unsigned char hits = 0;
	int raw = show_mux_controller_from_switches(switch_bits);

	if (raw == stable) {
		if (hits < 2u) {
			hits++;
		}
	} else {
		stable = raw;
		hits = 1u;
	}
	if (hits < 2u) {
		return SHOW_MUX_UNSTABLE;
	}
	return stable;
}

static const ShowProfile *profile_for_controller(int controller_id)
{
	switch (controller_id) {
	case OnChipController_RPGD:
		return &kProfileRpgd;
	case OnChipController_NeuralImitator:
		return &kProfileShortPl;
	case OnChipController_neural_controller_C:
	case OnChipController_neural_controller_LSTM_C:
	default:
		return &kProfileLong;
	}
}

void apply_show_profile_for_controller(int controller_id)
{
	const ShowProfile *p = profile_for_controller(controller_id);

	Motor_Stop();
	POLLING_PERIOD_MS = p->period_ms;
	SetControlUpdatePeriod(POLLING_PERIOD_MS);
	set_timesteps_for_derivative(p->derivative_n);
	set_derivative_median_len(p->derivative_median_len);
	set_fpga_deadzone_extrapolation(p->fpga_dz_extrapolation);

	MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = p->pwm_period;
	MOTOR_FULL_SCALE_SAFE = (int)(0.95f * (float)p->pwm_period + 0.5f);
	Motor_SetPwmPeriod(p->pwm_period);

	MOTOR_CORRECTION[0] = p->motor_correction[0];
	MOTOR_CORRECTION[1] = p->motor_correction[1];
	MOTOR_CORRECTION[2] = p->motor_correction[2];

	SliderTargetHalfLength = SLIDER_TARGET_HALF_LENGTH_SHOW;
	POSITION_ENCODER_RANGE = p->encoder_range;
	POSITION_NORMALIZATION_FACTOR = (TrackHalfLength * 2.0f) / POSITION_ENCODER_RANGE;

	secloc_controller_set_time_quantum((float)POLLING_PERIOD_MS / 1000.0f);
}
