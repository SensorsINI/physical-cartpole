/* 2026-09-02 go-to: on-chip AMP RPGD, 20 ms, 8 rollouts (4 per core), N=1. */
/* Source/config fingerprint: 0502c299bb481e2a */
#ifndef RPGD_ZYNQ_30MS_CONFIG_H
#define RPGD_ZYNQ_30MS_CONFIG_H

#include "rpgd_c/rpgd_cartpole.h"

#define RPGD_CONTROL_PERIOD_MS 20
#define RPGD_30MS_DERIVATIVE_STEPS 1

#define RPGD_30MS_CONFIG { \
    .mpc_horizon = 35, \
    .num_rollouts = 8, \
    .outer_its = 5, \
    .resamp_per = 10, \
    .period_interpolation_inducing_points = 4, \
    .intermediate_steps = 10, \
    .shift_previous = 1, \
    .sampling_distribution = 0, \
    .sample_whole_control_space = 0, \
    .warmup = 0, \
    .warmup_iterations = 250, \
    .num_threads = 1, \
    .reserve_threads = 1, \
    .seed = 123u, \
    .mpc_timestep = 0.02f, \
    .learning_rate = 0.0500000007f, \
    .adam_beta_1 = 0.899999976f, \
    .adam_beta_2 = 0.999000013f, \
    .adam_epsilon = 9.99999994e-09f, \
    .gradmax_clip = 5.0f, \
    .opt_keep_k_ratio = 0.75f, \
    .sample_stdev = 0.5f, \
    .sample_mean = 0.0f, \
    .uniform_dist_min = -0.800000012f, \
    .uniform_dist_max = 0.800000012f, \
    .action_low = -1.0f, \
    .action_high = 1.0f, \
    .k = 0.333333343f, \
    .m_cart = 0.230000004f, \
    .m_pole = 0.0869999975f, \
    .g = 9.81000042f, \
    .J_fric = 4.99999987e-05f, \
    .M_fric = 3.22000003f, \
    .L = 0.395000011f, \
    .u_max = 1.76999998f, \
    .track_half_length = 0.197999999f, \
    .dd_quadratic_weight_up = 200.0f, \
    .db_weight_up = 10000.0f, \
    .ep_weight_up = 20.0f, \
    .ekp_weight_up = 30.0f, \
    .cc_weight_up = 1.0f, \
    .vel_penalty_reg = 0.899999976f, \
    .R = 1.0f, \
    .permissible_track_fraction = 0.800000012f \
}

#endif
