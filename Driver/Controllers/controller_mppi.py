"""
Model Predictive Path Integral Controller
Based on Williams, Aldrich, Theodorou (2015)
"""

# Uncomment if you want to get interactive plots for MPPI in Pycharm on MacOS
# On other OS you have to chose a different interactive backend.
from matplotlib import use

import os
import matplotlib.pyplot as plt
import numpy as np
import yaml
import seaborn as sns

sns.set()
from CartPole.cartpole_model import TrackHalfLength
from CartPole.state_utilities import (
    ANGLE_IDX,
    ANGLED_IDX,
    POSITION_IDX,
    POSITIOND_IDX,
    create_cartpole_state,
)
from matplotlib.widgets import Slider
from numba import jit
from numpy.random import SFC64, Generator
from SI_Toolkit_ApplicationSpecificFiles.predictor_ODE import predictor_ODE
from scipy.interpolate import interp1d
from SI_Toolkit.TF.TF_Functions.predictor_autoregressive_tf import predictor_autoregressive_tf
from Controllers.template_controller import template_controller
from globals import *
import time as global_time
import tensorflow as tf

config = yaml.load(open(os.path.join("SI_Toolkit_ApplicationSpecificFiles", "config.yml"), "r"), Loader=yaml.FullLoader)

NET_NAME = config["modeling"]["NET_NAME"]
NET_TYPE = NET_NAME.split("-")[0]
CONTROLLER_CONFIG = "mppi_" + PREDICTOR

config = yaml.load(open("config.yml", "r"), Loader=yaml.FullLoader)
"""Timestep and sampling settings"""
dt = config["controller"][CONTROLLER_CONFIG]["dt"]
mpc_horizon = config["controller"][CONTROLLER_CONFIG]["mpc_horizon"]
mpc_samples = int(mpc_horizon / dt)  # Number of steps in MPC horizon
num_rollouts = config["controller"][CONTROLLER_CONFIG]["num_rollouts"]
predictor_type = config["controller"][CONTROLLER_CONFIG]["predictor_type"]

"""Perturbation factor"""
p_Q = config["controller"][CONTROLLER_CONFIG]["control_noise"]
dd_noise = ep_noise = ekp_noise = ekc_noise = cc_noise = config["controller"][CONTROLLER_CONFIG]["cost_noise"]
gui_dd = gui_ep = gui_ekp = gui_ekc = gui_cc = gui_ccrc = np.zeros(1, dtype=np.float32)

"""MPPI constants"""
R = config["controller"][CONTROLLER_CONFIG]["R"]
LBD = config["controller"][CONTROLLER_CONFIG]["LBD"]
NU = config["controller"][CONTROLLER_CONFIG]["NU"]
SQRTRHODTINV = config["controller"][CONTROLLER_CONFIG]["SQRTRHOINV"] * (1 / np.math.sqrt(dt))
#GAMMA = config["controller"][CONTROLLER_CONFIG]["GAMMA"]
SAMPLING_TYPE = config["controller"][CONTROLLER_CONFIG]["SAMPLING_TYPE"]

"""Define Predictor"""
if predictor_type == "Euler":
    predictor = predictor_ODE(horizon=mpc_samples, dt=dt, intermediate_steps=10)
elif predictor_type == "EulerTF":
    predictor = predictor_autoregressive_tf(horizon=mpc_samples, batch_size=num_rollouts, net_name='EulerTF', dt=dt, intermediate_steps=10)
elif predictor_type == "NeuralNet":
    predictor = predictor_autoregressive_tf(horizon=mpc_samples, batch_size=num_rollouts, net_name=NET_NAME)


"""Cost function helpers"""
@jit(nopython=True, cache=True, fastmath=True)
def E_kin_cart(positionD):
    """Compute penalty for kinetic energy of cart"""
    return positionD ** 2


@jit(nopython=True, cache=True, fastmath=True)
def E_kin_pol(angleD):
    """Compute penalty for kinetic energy of pole"""
    return angleD ** 2


@jit(nopython=True, cache=True, fastmath=True)
def E_pot_pole(angle):
    """Compute penalty for not balancing pole upright (penalize large angles)"""
    #return 0.25 * (1.0 - np.cos(angle)) ** 2
    return angle ** 2 / 10


@jit(nopython=True, cache=True, fastmath=True)
def distance_difference_cost(position, target_position):
    """Compute penalty for distance of cart to the target position"""
    return (np.abs(position - target_position) / (2.0 * TrackHalfLength)) ** 2


@jit(nopython=True, cache=True, fastmath=True)
def border_barrier(position, border_threshold):
    return np.where(np.abs(position) > border_threshold * TrackHalfLength, 1, 0)


@jit(nopython=True, cache=True, fastmath=True)
def control_change_rate_cost(delta_u):
    """Compute penalty of control jerk, i.e. difference to previous control input"""
    return (delta_u) ** 2
    #return np.diff(delta_u) ** 2


@jit(nopython=True, cache=True, fastmath=True)
def reward_weighted_average(S: np.ndarray, delta_u: np.ndarray):
    """Average the perturbations delta_u based on their desirability

    :param S: Array of rollout costs
    :type S: np.ndarray
    :param delta_u: Array of perturbations
    :type delta_u: np.ndarray
    :return: Gain to update the vector of nominal inputs by. Vector of length (horizon_steps)
    :rtype: np.ndarray
    """
    rho = np.min(S)  # for numerical stability
    exp_s = np.exp(-1.0 / LBD * (S - rho))
    a = np.sum(exp_s)
    b = np.sum(np.multiply(np.expand_dims(exp_s, 1), delta_u) / a, axis=0)
    return b


@jit(nopython=True, cache=True, fastmath=True)
def update_inputs(u, S, delta_u):
    """Reward-weighted in-place update of nominal control inputs according to the MPPI method.

    :param u: Sampling mean / warm started control inputs of size (,mpc_samples)
    :type u: np.ndarray
    :param S: Cost array of size (num_rollouts)
    :type S: np.ndarray
    :param delta_u: The input perturbations that had been used, shape (num_rollouts x mpc_samples)
    :type delta_u: np.ndarray
    """
    return u + reward_weighted_average(S, delta_u)


class controller_mppi(template_controller):
    """Controller implementing the Model Predictive Path Integral method (Williams et al. 2015)

    :param template_controller: Superclass describing the basic controller interface
    :type template_controller: abc.ABC
    """

    def __init__(self):

        SEED = config["controller"][CONTROLLER_CONFIG]["SEED"]
        self.rng_mppi = Generator(SFC64(SEED))
        self.rng_mppi_rnn = Generator(SFC64(SEED * 2))  # There are some random numbers used at warm up of rnn only. Separate rng prevents a shift

        self.controller_name = 'mppi-tf'
        self.angleErr = 0.0
        self.positionErr = 0.0
        self.ANGLE_TARGET = 0.0

        self.logging_stage_cost = config["controller"][CONTROLLER_CONFIG]["logging"]["stage_cost"]
        self.logging_cost_to_go_breakdown = config["controller"][CONTROLLER_CONFIG]["logging"]["cost_to_go_breakdown"]
        self.logging_average_cost_to_go = config["controller"][CONTROLLER_CONFIG]["logging"]["average_cost_to_go"]
        self.logging_trajectories = config["controller"][CONTROLLER_CONFIG]["logging"]["trajectories"]

        if self.logging_trajectories:
            use('TkAgg')

        # Cost Weights
        self.dd_weight = config["controller"][CONTROLLER_CONFIG]["stage_cost"]["dd_weight"] * (1 + dd_noise * self.rng_mppi.uniform(-1.0, 1.0))
        self.ep_weight = config["controller"][CONTROLLER_CONFIG]["stage_cost"]["ep_weight"] * (1 + ep_noise * self.rng_mppi.uniform(-1.0, 1.0))
        self.ekp_weight = config["controller"][CONTROLLER_CONFIG]["stage_cost"]["ekp_weight"] * (1 + ekp_noise * self.rng_mppi.uniform(-1.0, 1.0))
        self.ekc_weight = config["controller"][CONTROLLER_CONFIG]["stage_cost"]["ekc_weight"] * (1 + ekc_noise * self.rng_mppi.uniform(-1.0, 1.0))
        self.cc_weight = config["controller"][CONTROLLER_CONFIG]["stage_cost"]["cc_weight"] * (1 + cc_noise * self.rng_mppi.uniform(-1.0, 1.0))
        self.ccrc_weight = config["controller"][CONTROLLER_CONFIG]["stage_cost"]["ccrc_weight"]
        self.border_safety_weight = config["controller"][CONTROLLER_CONFIG]["stage_cost"]["border_safety"]
        self.border_threshold = config["controller"][CONTROLLER_CONFIG]["stage_cost"]["border_threshold"]
        self.terminal_angle_weight = config["controller"][CONTROLLER_CONFIG]["terminal_cost"]["angle"]
        self.terminal_position_weight = config["controller"][CONTROLLER_CONFIG]["terminal_cost"]["position"]
        self.terminal_goal_angle = config["controller"][CONTROLLER_CONFIG]["terminal_goal"]["angle"]
        self.terminal_goal_position = config["controller"][CONTROLLER_CONFIG]["terminal_goal"]["position"]

        # State of the cart
        self.state = create_cartpole_state()
        self.target_position = 0.0
        self.rho_sqrt_inv = 0.01
        self.control_enabled = True

        self.s_horizon = np.zeros((), dtype=np.float32)
        self.u = np.zeros((mpc_samples), dtype=np.float32)
        self.u_prev = np.zeros_like(self.u, dtype=np.float32)
        self.delta_u = np.zeros((num_rollouts, mpc_samples), dtype=np.float32)
        self.rollout_costs = np.zeros((num_rollouts), dtype=np.float32)

        self.num_rollouts = num_rollouts
        self.horizon = mpc_samples
        self.predictor_type = predictor_type

        # Logging
        self.logs = {
            # Cost To Go
            "cost_to_go_breakdown": [],
            "cost_to_go_breakdown_rollouts": [],
            "average_cost_to_go": [],
            "stage_cost_breakdown": [],
            # Rollouts
            "rollout_states": [],
            "rollout_inputs": [],
            "rollout_costs": [],
            "rollout_index": [],
            # Realized Trajectory
            "realized_trajectory": [],
            "realized_costs": [],
            "cost_to_go": [],
            # Nominal Trajectory
            "nominal_trajectory": [],
            "nominal_costs": [],
            # Reference Trajectory
            "reference_trajectory": [],
        }

    def initialize_perturbations(self, stdev=1.0, sampling_type=None):
        """Sample an array of control perturbations delta_u. Samples for two distinct rollouts are always independent

        :param stdev: standard deviation of samples if Gaussian, defaults to 1.0
        :param sampling_type: defaults to None, can be one of
            - "random_walk" - The next horizon step's perturbation is correlated with the previous one
            - "uniform" - Draw uniformly distributed samples between -1.0 and 1.0
            - "repeated" - Sample only one perturbation per rollout, apply it repeatedly over the course of the rollout
            - "interpolated" - Sample a new independent perturbation every 10th MPC horizon step. Interpolate in between the samples
            - "iid" - Sample independent and identically distributed samples of a Gaussian distribution
        :return: Independent perturbation samples of shape (num_rollouts x horizon_steps)
        """
        """
        Return a numpy array with the perturbations delta_u.
        If random_walk is false, initialize with independent Gaussian samples
        If random_walk is true, each row represents a 1D random walk with Gaussian steps.
        """
        if sampling_type == "random_walk":
            delta_u = np.empty((num_rollouts, mpc_samples), dtype=np.float32)
            delta_u[:, 0] = stdev * self.rng_mppi.standard_normal(
                size=(num_rollouts,), dtype=np.float32
            )
            for i in range(1, mpc_samples):
                delta_u[:, i] = delta_u[:, i - 1] + stdev * self.rng_mppi.standard_normal(
                    size=(num_rollouts,), dtype=np.float32
                )
        elif sampling_type == "uniform":
            delta_u = np.empty((num_rollouts, mpc_samples), dtype=np.float32)
            for i in range(0, mpc_samples):
                delta_u[:, i] = self.rng_mppi.uniform(
                    low=-1.0, high=1.0, size=(num_rollouts,)
                ).astype(np.float32)
        elif sampling_type == "repeated":
            delta_u = np.tile(
                stdev * self.rng_mppi.standard_normal(size=(num_rollouts, 1), dtype=np.float32),
                (1, mpc_samples),
            )
        elif sampling_type == "interpolated":
            step = 10
            range_stop = int(np.ceil((mpc_samples) / step) * step) + 1
            t = np.arange(start=0, stop=range_stop, step=step)
            t_interp = np.arange(start=0, stop=range_stop, step=1)
            t_interp = np.delete(t_interp, t)
            delta_u = np.zeros(shape=(num_rollouts, range_stop), dtype=np.float32)
            delta_u[:, t] = stdev * self.rng_mppi.standard_normal(size=(num_rollouts, t.size), dtype=np.float32)
            f = interp1d(t, delta_u[:, t])
            delta_u[:, t_interp] = f(t_interp)
            delta_u = delta_u[:, :mpc_samples]
        else:
            delta_u = stdev * self.rng_mppi.standard_normal(size=(num_rollouts, mpc_samples), dtype=np.float32)

        return delta_u

    def trajectory_rollouts(self, s, u, delta_u, target_position):
        """Sample thousands of rollouts using system model. Compute cost-weighted control update. Log states and costs if specified.

        :param s: Current state of the system
        :param S_tilde_k: Placeholder array to store the cost of each rollout trajectory
        :param u: Vector of nominal inputs computed in previous iteration
        :param delta_u: Array containing all input perturbation samples. Shape (num_rollouts x horizon_steps)
        :param u_prev: Array with nominal inputs from previous iteration. Used to compute cost of control change

        :return: rollout_costs - Array filled with a cost for each rollout trajectory
        """
        s = predictor.predict_tf(tf.convert_to_tensor(s, dtype=tf.float32), tf.convert_to_tensor(u + delta_u, dtype=tf.float32)).numpy()

        # Compute costs
        total_cost, dd, ep, ekp, ekc, cc, ccrc, border = self.stage_cost(s, u, delta_u, target_position)
        terminal_cost = self.terminal_cost(s, target_position)
        rollout_costs = np.sum(total_cost, axis=1) + terminal_cost

        # Pass costs to GUI popup window (0.3ms)
        global gui_dd, gui_ep, gui_ekp, gui_ekc, gui_cc, gui_ccrc
        gui_dd, gui_ep, gui_ekp, gui_ekc, gui_cc, gui_ccrc = (np.mean(dd), np.mean(ep), np.mean(ekp), np.mean(ekc), np.mean(cc), np.mean(ccrc))

        if self.logging_trajectories:
            self.logs.get("rollout_states").append(s)
            self.logs.get("rollout_inputs").append(u + delta_u)
            self.logs.get("rollout_costs").append(total_cost)
        if self.logging_cost_to_go_breakdown:
            self.cost_to_go_breakdown_rollouts = np.stack([rollout_costs, terminal_cost, dd.sum(axis=1), ep.sum(axis=1), ekp.sum(axis=1), ekc.sum(axis=1), cc.sum(axis=1), ccrc.sum(axis=1), border.sum(axis=1)], axis=0)

        return rollout_costs

    def stage_cost(self, s, u, delta_u, target_position):
        dd = self.dd_weight * distance_difference_cost(s[..., POSITION_IDX], target_position)
        ep = self.ep_weight * E_pot_pole(s[..., ANGLE_IDX])
        ekp = self.ekp_weight * E_kin_pol(s[..., ANGLED_IDX])
        ekc = self.ekc_weight * E_kin_cart(s[..., POSITIOND_IDX])
        cc = self.cc_weight * ((0.5 * (1 - 1.0 / NU) * R * delta_u ** 2 + R * u * delta_u + 0.5 * R * u ** 2))
        ccrc = self.ccrc_weight * control_change_rate_cost(delta_u)
        border = self.border_safety_weight * border_barrier(s[..., POSITION_IDX], self.border_threshold)

        total_cost = dd + ep + ekp + ekc + cc + ccrc + border

        return total_cost, dd, ep, ekp, ekc, cc, ccrc, border

    def sigmoid(self, x):
        return 1/(1 + np.exp(-20 * x))

    def terminal_cost(self, s, target_position):
        """Calculate terminal cost of a set of trajectories

        Williams et al use an indicator function type of terminal cost in
        "Information theoretic MPC for model-based reinforcement learning"

        TODO: Try a quadratic terminal cost => Use the LQR terminal cost term obtained
        by linearizing the system around the unstable equilibrium.

        :param s: Reference to numpy array of states of all rollouts
        :param target_position: Target position to move the cart to
        :return: One terminal cost per rollout
        """
        terminal_states = s[:, -1, :]
        #terminal_cost = self.terminal_angle_weight * (np.abs(terminal_states[:, ANGLE_IDX]) > (self.terminal_goal_angle * np.pi))
        #terminal_cost += self.terminal_position_weight * (np.abs(terminal_states[:, POSITION_IDX] - target_position) > self.terminal_goal_position * TrackHalfLength)
        terminal_cost = self.terminal_angle_weight * self.sigmoid(np.abs(terminal_states[:, ANGLE_IDX]) - (self.terminal_goal_angle * np.pi))
        terminal_cost += self.terminal_position_weight * self.sigmoid(np.abs(terminal_states[:, POSITION_IDX] - target_position) - self.terminal_goal_position * TrackHalfLength)

        return terminal_cost

    # Step (Euler: 13.1, RNN:15.8ms)
    def step(self, s, target_position, time=None):
        """Perform controller step

        :param s: State passed to controller after system has evolved for one step
        :param target_position: Target position where the cart should move to
        :param time: Time in seconds that has passed in the current experiment, defaults to None
        :return: A normed control value in the range [-1.0, 1.0]
        """

        self.state = s
        self.target_position = np.float32(target_position)

        # Initialize Perturbations
        self.delta_u = self.initialize_perturbations(stdev=SQRTRHODTINV, sampling_type=SAMPLING_TYPE)
        self.delta_u = np.clip(self.delta_u, -1.0 - self.u, 1.0 - self.u, dtype=np.float32)

        # Trajectory Rollouts (Euler: 10.2ms, RNN:14.5ms)
        start = global_time.time()
        self.rollout_costs = self.trajectory_rollouts(self.state, self.u, self.delta_u, self.target_position)
        performance_measurement[1] = global_time.time() - start

        # Update inputs with weighted perturbations
        self.rollout_u = self.u + self.delta_u
        self.u = update_inputs(self.u, self.rollout_costs, self.delta_u)
        self.u = np.clip(self.u, -1.0, 1.0, dtype=np.float32)

        # Nominal Realization for Plotting
        if self.logging_trajectories:
            self.logs.get("realized_trajectory").append(np.append(self.state, np.expand_dims(self.u[0], axis=0), axis=-1))
            self.logs.get("reference_trajectory").append(target_position)
            self.current_cost = self.stage_cost(self.state, self.u[0], self.u_prev[1] - self.u[0], target_position)
            self.logs.get("realized_costs").append(self.current_cost)
            self.logs.get("cost_to_go").append(self.rollout_costs)
            chosen_rollout_index = np.linalg.norm(self.rollout_u - self.u, axis=1).argmin()
            self.logs.get("rollout_index").append(chosen_rollout_index)

        # Average Cost to Go
        if self.logging_average_cost_to_go:
            self.logs.get("average_cost_to_go").append(np.mean(self.rollout_costs))

        # Cost To Go Breakdown
        if self.logging_cost_to_go_breakdown:
            chosen_rollout_index = np.linalg.norm(self.rollout_u - self.u, axis=1).argmin()
            self.logs.get("cost_to_go_breakdown").append(np.copy(self.cost_to_go_breakdown_rollouts[:,chosen_rollout_index]))

        # Stage Cost Breakdown
        if self.logging_stage_cost:
            self.current_cost = self.stage_cost(self.state, self.u[0], self.u_prev[1] - self.u[0], target_position)
            self.logs.get("stage_cost_breakdown").append(np.copy(self.current_cost))

        Q = self.u[0]
        Q = np.float32(Q * (1 + p_Q * self.rng_mppi.uniform(-1.0, 1.0)))
        Q = np.clip(Q, -1.0, 1.0, dtype=np.float32)

        # Preserve current series of inputs
        self.u_prev = np.copy(self.u)

        # Index-shift inputs
        self.u[:-1] = self.u[1:]
        self.u[-1] = 0

        # Prepare predictor for next timestep (0.1ms)
        predictor.update_internal_state_tf(tf.convert_to_tensor(Q, dtype=tf.float32))

        return Q

    def print_help(self):
        print('*** Controller Informations here ***')

    def controller_report(self):
        print('\nCreating Plots.')

        if self.logging_trajectories:
            ## Plot Trajectories
            rollout_states = np.stack(self.logs.get("rollout_states"), axis=0)
            rollout_inputs = np.stack(self.logs.get("rollout_inputs"), axis=0)
            rollout_costs = np.stack(self.logs.get("rollout_costs"), axis=0)
            rollout_index = np.stack(self.logs.get("rollout_index"), axis=0)

            nominal_states = rollout_states[np.arange(rollout_states.shape[0]), rollout_index, ...].squeeze()
            nominal_costs = rollout_costs[np.arange(rollout_costs.shape[0]), rollout_index, ...].squeeze()
            nominal_inputs = rollout_inputs[np.arange(rollout_inputs.shape[0]), rollout_index, ...].squeeze()

            realized_states = np.stack(self.logs.get("realized_trajectory"), axis=0)
            realized_costs = np.stack(self.logs.get("realized_costs"), axis=0)

            reference_trajectory = np.stack(self.logs.get("reference_trajectory")[:-1], axis=0)
            cost_to_go = np.stack(self.logs.get("cost_to_go"), axis=0)

            # Create figure
            fig, axs = plt.subplots(nrows=6, ncols=1, num=1, sharex=True, gridspec_kw={"bottom": 0.15, "left": 0.1, "right": 0.84, "top": 0.95}, figsize=(16, 12))


            # Create time slider
            slider_axis = plt.axes([0.15, 0.02, 0.7, 0.03])
            slider = Slider(slider_axis, "timestep", 1, np.shape(rollout_states)[0], valinit=1, valstep=1)

            # Normalize cost to go to use as opacity in plot
            cost_to_go = np.divide(cost_to_go.T, np.max(np.abs(cost_to_go), axis=1)).T

            # This function updates the plot when a new iteration is selected
            def update_plot(i):
                rollouts = [rollout_states[i - 1, :, :, POSITION_IDX], rollout_states[i - 1, :, :, POSITIOND_IDX], rollout_states[i - 1, :, :, ANGLE_IDX], rollout_states[i - 1, :, :, ANGLED_IDX], rollout_inputs[i - 1], rollout_costs[i - 1]]
                realized_trajectories = [realized_states[:, POSITION_IDX], realized_states[:, POSITIOND_IDX], realized_states[:, ANGLE_IDX], realized_states[:, ANGLED_IDX], realized_states[:, -1], realized_costs[..., 0]]
                nominal_trajectories = [nominal_states[i - 1, :, POSITION_IDX], nominal_states[i - 1, :, POSITIOND_IDX], nominal_states[i - 1, :, ANGLE_IDX], nominal_states[i - 1, :, ANGLED_IDX], nominal_inputs[i - 1, :], nominal_costs[i - 1]]

                for j, ax in enumerate(axs):
                    ax.clear()
                    iteration = i - 1
                    costs = cost_to_go[i - 1, :]

                    if j==5:
                        ax.set_yscale('log')

                    # Rollouts (for current slider position)
                    rollout = rollouts[j]
                    for l in range(0, self.num_rollouts, 5):
                        ax.plot(
                            (iteration + np.arange(0, self.horizon)) * dt,
                            rollout[l, :],
                            alpha=0.15 * (1 - 0.3 * costs[l]) ** 2, color='limegreen', linestyle='-', linewidth=1, marker='.', markersize=3
                        )

                    # Realized Trajectories (for whole timeline)
                    realized_trajectory = realized_trajectories[j]
                    ax.plot(
                        np.arange(0, np.shape(realized_states)[0]) * dt,
                        realized_trajectory,
                        linestyle='--', linewidth=1, color='green', label='realized trajectory', marker='.', markersize=3
                    )

                    # Nominal Trajectories (for current slider position)
                    nominal_trajectory = nominal_trajectories[j]
                    ax.plot(
                        ((i - 1) + np.arange(0, nominal_trajectory.shape[0])) * dt,
                        nominal_trajectory,
                        linestyle='--', linewidth=1, color='red', label='nominal trajectory', marker='.', markersize=3
                    )

                # Limits
                axs[0].set_xlim(0, np.shape(realized_states)[0] * dt)
                axs[0].set_ylim(-TrackHalfLength * 1.05, TrackHalfLength * 1.05)

                # Labels
                axs[0].set_title("Monte Carlo Rollouts")
                axs[0].set_ylabel("position (m)", fontsize='x-small')
                axs[1].set_ylabel("velocity (m/s)", fontsize='x-small')
                axs[2].set_ylabel("angle (deg)", fontsize='x-small')
                axs[3].set_ylabel("angular rate (deg/s)", fontsize='x-small')
                axs[4].set_ylabel("input Q ∈ [-1,1]", fontsize='x-small')
                axs[5].set_ylabel("total cost", fontsize='x-small')
                axs[5].set_xlabel("time (s)", fontsize='small')

                # Legends
                axs[0].legend(loc="upper left", bbox_to_anchor=(1, 0, 0.16, 1), fontsize='xx-small')

            update_plot(1)
            slider.on_changed(update_plot)
            plt.show()

        ### Average Cost to Go
        if self.logging_average_cost_to_go:
            average_cost_to_go = np.stack(self.logs.get("average_cost_to_go"), axis=0)
            time_axis = dt * np.arange(start=0, stop=np.shape(average_cost_to_go)[0])
            plt.figure(num=2)
            plt.semilogy(time_axis, average_cost_to_go, marker='.', markersize=8)
            plt.title("Average Cost-to-go (over all rollouts)")
            plt.xlabel("time in s")
            plt.ylabel("Cost")
            plt.legend(fontsize='xx-small')
            plt.show()

        ### Cost to Go Breakdown
        if self.logging_cost_to_go_breakdown:
            cost_to_go_breakdown = np.stack(self.logs.get("cost_to_go_breakdown"), axis=0)
            time_axis = dt * np.arange(start=0, stop=np.shape(cost_to_go_breakdown)[0])
            labels = ['Total Cost to Go', 'Terminal Cost', 'Cart Distance', 'Pole Potential', 'Pole Kinetic', 'Cart Kinetic', 'Control Cost', 'Control Change Rate', 'Border Safety']

            plt.figure(num=3)
            for i in range(9):
                if cost_to_go_breakdown[:, i].max() > 1:
                    plt.semilogy(time_axis, cost_to_go_breakdown[:, i], label=labels[i], marker='.', markersize=2, linewidth=1)
            plt.title("Cost-to-go Breakdown (of selected rollout)")
            plt.xlabel("time in s")
            plt.ylabel("Cost")
            plt.legend(loc='lower left', fontsize='xx-small')
            plt.show()

        ### Stage Cost Breakdown
        if self.logging_stage_cost:
            realized_costs = np.stack(self.logs.get("stage_cost_breakdown"), axis=0)
            time_axis = dt * np.arange(start=0, stop=len(realized_costs[:, 0]))
            labels = ['Total Cost', 'Cart Distance', 'Pole Potential', 'Pole Kinetic', 'Cart Kinetic', 'Control Cost', 'Control Change Rate', 'Border Safety']

            plt.figure(num=4)
            for i in range(8):
                if realized_costs[:, i].max() > 1:
                    plt.semilogy(time_axis, realized_costs[:, i], label=labels[i], marker='.', markersize=2, linewidth=1)

            plt.title("Stage Cost Breakdown (of realized states)")
            plt.xlabel("time in s")
            plt.ylabel("Cost")
            plt.legend(loc='lower left', fontsize='xx-small')
            plt.show()

        self.logs = {
            # Cost To Go
            "cost_to_go_breakdown": [],
            "cost_to_go_breakdown_rollouts": [],
            "average_cost_to_go": [],
            "stage_cost_breakdown": [],
            # Rollouts
            "rollout_states": [],
            "rollout_inputs": [],
            "rollout_costs": [],
            "rollout_index": [],
            # Realized Trajectory
            "realized_trajectory": [],
            "realized_costs": [],
            "cost_to_go": [],
            # Nominal Trajectory
            "nominal_trajectory": [],
            "nominal_costs": [],
            # Reference Trajectory
            "reference_trajectory": [],
        }

    # Optionally: reset the controller after an experiment
    # May be useful for stateful controllers, like these containing RNN,
    # To reload the hidden states e.g. if the controller went unstable in the previous run.
    # It is called after an experiment,
    # but only if the controller is supposed to be reused without reloading (e.g. in GUI)
    def controller_reset(self):
        try:
            self.warm_up_countdown = self.wash_out_len
            # TODO: Not sure if this works for predictor autoregressive tf
            predictor.net.reset_states()
        except:
            pass
