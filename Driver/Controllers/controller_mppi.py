"""
Model Predictive Path Integral Controller
Based on Williams, Aldrich, Theodorou (2015)
"""

# Uncomment if you want to get interactive plots for MPPI in Pycharm on MacOS
# On other OS you have to chose a different interactive backend.
from matplotlib import use
use('TkAgg')
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

config = yaml.load(open("config.yml", "r"), Loader=yaml.FullLoader)
"""Timestep and sampling settings"""
dt = config["controller"]["mppi"]["dt"]
mpc_horizon = config["controller"]["mppi"]["mpc_horizon"]
mpc_samples = int(mpc_horizon / dt)  # Number of steps in MPC horizon
num_rollouts = config["controller"]["mppi"]["num_rollouts"]
predictor_type = config["controller"]["mppi"]["predictor_type"]


"""Parameters weighting the different cost components"""
dd_weight = config["controller"]["mppi"]["dd_weight"]
ep_weight = config["controller"]["mppi"]["ep_weight"]
ekp_weight = config["controller"]["mppi"]["ekp_weight"]
ekc_weight = config["controller"]["mppi"]["ekc_weight"]
cc_weight = config["controller"]["mppi"]["cc_weight"]
ccrc_weight = config["controller"]["mppi"]["ccrc_weight"]

"""Perturbation factor"""
p_Q = config["controller"]["mppi"]["control_noise"]
dd_noise = ep_noise = ekp_noise = ekc_noise = cc_noise = config["controller"]["mppi"]["cost_noise"]
gui_dd = gui_ep = gui_ekp = gui_ekc = gui_cc = gui_ccrc = np.zeros(1, dtype=np.float32)


"""MPPI constants"""
R = config["controller"]["mppi"]["R"]
LBD = config["controller"]["mppi"]["LBD"]
NU = config["controller"]["mppi"]["NU"]
SQRTRHODTINV = config["controller"]["mppi"]["SQRTRHOINV"] * (1 / np.math.sqrt(dt))
GAMMA = config["controller"]["mppi"]["GAMMA"]
SAMPLING_TYPE = config["controller"]["mppi"]["SAMPLING_TYPE"]


"""Define Predictor"""
if predictor_type == "Euler":
    predictor = predictor_ODE(horizon=mpc_samples, dt=dt, intermediate_steps=10)
elif predictor_type == "EulerTF":
    predictor = predictor_autoregressive_tf(horizon=mpc_samples, batch_size=num_rollouts, net_name='EulerTF', dt=dt, intermediate_steps=10)
elif predictor_type == "NeuralNet":
    predictor = predictor_autoregressive_tf(horizon=mpc_samples, batch_size=num_rollouts, net_name=NET_NAME)

predictor_ground_truth = predictor_autoregressive_tf(horizon=mpc_samples, batch_size=1, net_name='EulerTF', dt=dt, intermediate_steps=10)


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
    return 0.25 * (1.0 - np.cos(angle)) ** 2


@jit(nopython=True, cache=True, fastmath=True)
def distance_difference_cost(position, target_position):
    """Compute penalty for distance of cart to the target position"""
    return 1e1 * (np.abs(position - target_position) / (2.0 * TrackHalfLength)) ** 2 + (
        np.abs(position) > 0.95 * TrackHalfLength
    ) * 1.0e6  # Soft constraint: Do not crash into border


@jit(nopython=True, cache=True, fastmath=True)
def control_change_rate_cost(delta_u):
    """Compute penalty of control jerk, i.e. difference to previous control input"""
    return (delta_u) ** 2

@jit(nopython=True, cache=True, fastmath=True)
def terminal_cost(s, target_position) -> np.ndarray:
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
    terminal_cost = 10000 * (
        (np.abs(terminal_states[:, ANGLE_IDX]) > 0.2)
        | (
            np.abs(terminal_states[:, POSITION_IDX] - target_position)
            > 0.1 * TrackHalfLength
        )
    )
    return terminal_cost


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

        SEED = config["controller"]["mppi"]["SEED"]
        self.rng_mppi = Generator(SFC64(SEED))
        self.rng_mppi_rnn = Generator(SFC64(SEED*2)) # There are some random numbers used at warm up of rnn only. Separate rng prevents a shift

        self.controller_name = 'mppi-tf'
        self.angleErr = 0.0
        self.positionErr = 0.0
        self.ANGLE_TARGET = 0.0

        self.logging = config["controller"]["mppi"]["LOGGING"]


        global dd_weight, ep_weight, ekp_weight, ekc_weight, cc_weight
        dd_weight = dd_weight * (1 + dd_noise * self.rng_mppi.uniform(-1.0, 1.0))
        ep_weight = ep_weight * (1 + ep_noise * self.rng_mppi.uniform(-1.0, 1.0))
        ekp_weight = ekp_weight * (1 + ekp_noise * self.rng_mppi.uniform(-1.0, 1.0))
        ekc_weight = ekc_weight * (1 + ekc_noise * self.rng_mppi.uniform(-1.0, 1.0))
        cc_weight = cc_weight * (1 + cc_noise * self.rng_mppi.uniform(-1.0, 1.0))

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

        """Init logging variables"""
        # Save average cost for each cost component
        self.logs = {
            # Cost Breakdown
            "cost_to_go": [],
            "cost_breakdown": {
                "total_cost": [],
                "cost_dd": [],
                "cost_ep": [],
                "cost_ekp": [],
                "cost_ekc": [],
                "cost_cc": [],
                "cost_ccrc": [],
            },
            # Rollouts
            "rollout_states": [],
            "rollout_inputs": [],
            "rollout_costs": [],
            # Realized Trajectory
            "realized_trajectory": [],
            "realized_costs": [],
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
        start = global_time.time()
        s = predictor.predict_tf(tf.convert_to_tensor(s, dtype=tf.float32), tf.convert_to_tensor(u + delta_u, dtype=tf.float32)).numpy()
        performance_measurement[2] = global_time.time() - start

        # Compute costs
        total_cost, dd, ep, ekp, ekc, cc, ccrc = self.stage_cost(s, u, delta_u, target_position)
        rollout_costs = np.sum(total_cost, axis=1)
        rollout_costs += terminal_cost(s, target_position)

        # Pass costs to GUI popup window (0.3ms)
        global gui_dd, gui_ep, gui_ekp, gui_ekc, gui_cc, gui_ccrc
        gui_dd, gui_ep, gui_ekp, gui_ekc, gui_cc, gui_ccrc = (np.mean(dd), np.mean(ep), np.mean(ekp), np.mean(ekc), np.mean(cc), np.mean(ccrc))

        if self.logging:
            self.logs.get("rollout_states").append(np.copy(s))
            self.logs.get("rollout_inputs").append(np.copy(u + delta_u))
            self.logs.get("rollout_costs").append(np.copy(total_cost))

        return rollout_costs

    def stage_cost(self, s, u, delta_u, target_position):
        dd = dd_weight * distance_difference_cost(s[..., POSITION_IDX], target_position)
        ep = ep_weight * E_pot_pole(s[..., ANGLE_IDX])
        ekp = ekp_weight * E_kin_pol(s[..., ANGLED_IDX])
        ekc = ekc_weight * E_kin_cart(s[..., POSITIOND_IDX])
        cc = cc_weight * ((0.5 * (1 - 1.0 / NU) * R * delta_u ** 2 + R * u * delta_u + 0.5 * R * u ** 2))
        ccrc = ccrc_weight * control_change_rate_cost(delta_u)

        total_cost = dd + ep + ekp + ekc + cc + ccrc

        return total_cost, dd, ep, ekp, ekc, cc, ccrc

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
        self.delta_u = np.clip(self.delta_u, -1.0-self.u, 1.0-self.u, dtype=np.float32)

        # Trajectory Rollouts (Euler: 10.2ms, RNN:14.5ms)
        start = global_time.time()
        self.rollout_costs = self.trajectory_rollouts(self.state, self.u, self.delta_u, self.target_position)
        performance_measurement[1] = global_time.time() - start

        # Update inputs with weighted perturbations
        self.u = update_inputs(self.u, self.rollout_costs, self.delta_u)
        self.u = np.clip(self.u, -1.0, 1.0, dtype=np.float32)

        # Nominal Realization for Plotting
        if self.logging:
            self.logs.get("cost_to_go").append(np.copy(self.rollout_costs))
            predictor_ground_truth.setup(initial_state=np.expand_dims(self.state, axis=0), prediction_denorm=True)
            nominal_trajectory = predictor_ground_truth.predict(np.expand_dims(self.u, axis=0))[0]
            self.logs.get("nominal_trajectory").append(np.copy(nominal_trajectory[1:]))
            self.logs.get("nominal_costs").append(np.copy(self.stage_cost(nominal_trajectory[1:,:], self.u, self.u_prev-self.u, target_position)))
            self.logs.get("realized_trajectory").append(np.copy(np.append(self.state, np.expand_dims(self.u[0], axis=0), axis=-1)))
            self.current_cost = self.stage_cost(self.state, self.u[0], self.u_prev[1] - self.u[0], target_position)
            self.logs.get("realized_costs").append(np.copy(self.current_cost))
            self.logs.get("reference_trajectory").append(np.copy(target_position))

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

    def update_control_vector(self):
        """
        MPPI stores a vector of best-guess-so-far control inputs for future steps.
        When adjusting the horizon length, need to adjust this vector too.
        Init with zeros when lengthening, and slice when shortening horizon.
        """
        update_length = min(mpc_samples, self.u.size)
        u_new = np.zeros((mpc_samples), dtype=np.float32)
        u_new[:update_length] = self.u[:update_length]
        self.u = u_new
        self.u_prev = np.copy(self.u)

    def print_help(self):
        print('*** Controller Informations here ***')

    def controller_report(self, avg_cost_report=False, detail_cost_report=True, trajectory_report =True):
        if self.logging and self.logs.get("cost_to_go"):
            print('\nCreating Plots.')
            ctglgs = np.stack(self.logs.get("cost_to_go"), axis=0)
            NUM_ITERATIONS = np.shape(ctglgs)[0]

            ### Plot Cost-to-Go
            if avg_cost_report:
                time_axis = dt * np.arange(start=0, stop=np.shape(ctglgs)[0])
                plt.figure(num=2, figsize=(16, 9))
                plt.semilogy(time_axis, np.mean(ctglgs, axis=1), marker='.', markersize=8)
                plt.title("Cost-to-go per Timestep")
                plt.xlabel("time (s)")
                plt.ylabel("Average Running Cost")

            ### Plot Cost Breakdown
            if detail_cost_report:
                realized_costs  = np.stack(self.logs.get("realized_costs"), axis=0)

                time_axis = dt * np.arange(start=0, stop=NUM_ITERATIONS)
                labels = ['Total Cost', 'Cart Distance', 'Pole Potential', 'Pole Kinetic', 'Cart Kinetic', 'Control Cost', 'Control Change Rate']

                plt.figure(num=3, figsize=(16, 9))
                for i in range(7):
                    plt.semilogy(time_axis, realized_costs[:,i], label=labels[i], marker='.', markersize=6)

                plt.title("Cost component breakdown")
                plt.xlabel("time (s)")
                plt.ylabel("total horizon cost")
                plt.legend()

            ## Plot Trajectories
            if trajectory_report:
                # Prepare data
                rollout_states  = np.stack(self.logs.get("rollout_states"), axis=0)             # shape(slgs) = ITERATIONS x num_rollouts x mpc_samples x STATE_VARIABLES
                rollout_inputs  = np.stack(self.logs.get("rollout_inputs"), axis=0)             # shape(slgs) = ITERATIONS x num_rollouts x mpc_samples
                rollout_costs   = np.stack(self.logs.get("rollout_costs"), axis=0)              # shape(slgs) = ITERATIONS x num_rollouts x mpc_samples
                nominal_states = np.stack(self.logs.get("nominal_trajectory"), axis=0)          # shape = ITERATIONS x mpc_horizon x STATE_VARIABLES
                nominal_costs = np.stack(self.logs.get("nominal_costs"), axis=0)
                realized_states = np.stack(self.logs.get("realized_trajectory"), axis=0)        # shape = ITERATIONS x STATE_VARIABLES
                realized_costs  = np.stack(self.logs.get("realized_costs"), axis=0)
                reference_trajectory = np.stack(self.logs.get("reference_trajectory")[:-1], axis=0)  # shape(trgtlgs) = ITERATIONS x [position]

                # For each rollout, calculate what the nominal trajectory would be using the known true model
                # This can uncover if the model used makes inaccurate predictions
                # shape(true_nominal_rollouts) = ITERATIONS x mpc_horizon x [position, positionD, angle, angleD]
                #predictor_ground_truth.setup(np.copy(nrlgs[:, 0, :]), prediction_denorm=True)
                #true_nominal_rollouts = predictor_ground_truth.predict(iplgs)[:, :-1, :]

                # Create figure
                fig, axs = plt.subplots(nrows=6, ncols=1, num=5, figsize=(16, 12), sharex=True, gridspec_kw={"bottom": 0.15, "left": 0.1, "right": 0.84, "top": 0.95})

                # Create time slider
                slider_axis = plt.axes([0.15, 0.02, 0.7, 0.03])
                slider = Slider( slider_axis, "timestep", 1, np.shape(rollout_states)[0], valinit=1, valstep=1 )

                # Normalize cost to go to use as opacity in plot
                # shape(ctglgs) = ITERATIONS x num_rollouts
                ctglgs = np.divide(ctglgs.T, np.max(np.abs(ctglgs), axis=1)).T

                # This function updates the plot when a new iteration is selected
                def update_plot(i):
                    rollouts = [rollout_states[i - 1, :, :, POSITION_IDX], rollout_states[i - 1, :, :, POSITIOND_IDX], rollout_states[i - 1, :, :, ANGLE_IDX], rollout_states[i - 1, :, :, ANGLED_IDX], rollout_inputs[i-1], rollout_costs[i-1]]
                    realized_trajectories = [realized_states[:, POSITION_IDX], realized_states[:, POSITIOND_IDX], realized_states[:, ANGLE_IDX], realized_states[:, ANGLED_IDX], realized_states[:, -1], realized_costs[..., 0]]
                    nominal_trajectories = [nominal_states[i - 1, :, POSITION_IDX], nominal_states[i - 1, :, POSITIOND_IDX], nominal_states[i - 1, :, ANGLE_IDX], nominal_states[i - 1, :, ANGLED_IDX], nominal_states[i - 1, :, -1], nominal_costs[i-1, 0]]

                    for j, ax in enumerate(axs):
                        ax.clear()
                        iteration = i - 1
                        costs = ctglgs[i - 1, :]

                        # Rollouts (for current slider position)
                        rollout = rollouts[j]
                        for l in range(0, self.num_rollouts, 5):
                            ax.plot(
                                (iteration + np.arange(0, self.horizon)) * dt,
                                rollout[l, :],
                                alpha=0.15 * (1 - 0.3 * costs[l]) ** 2, color='limegreen', linestyle='-', linewidth=1, marker = '.', markersize=3
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
                            ((i - 1) + np.arange(0, np.shape(nominal_states)[1])) * dt,
                            nominal_trajectory,
                            linestyle='--', linewidth=1, color='red', label='nominal trajectory', marker='.', markersize=3
                        )

                    # Limits
                    axs[0].set_xlim(0, np.shape(realized_states)[0] * dt)
                    axs[0].set_ylim(-TrackHalfLength * 1.05, TrackHalfLength * 1.05)

                    # Labels
                    axs[0].set_title("Monte Carlo Rollouts")
                    axs[0].set_ylabel("position (m)")
                    axs[1].set_ylabel("velocity (m/s)")
                    axs[2].set_ylabel("angle (deg)")
                    axs[3].set_ylabel("angular rate (deg/s)")
                    axs[4].set_ylabel("input Q ∈ [-1,1]")
                    axs[5].set_ylabel("total cost")
                    axs[5].set_xlabel("time (s)")

                    # Legends
                    axs[0].legend(loc="upper left", bbox_to_anchor=(1, 0, 0.16, 1))

                # Draw first iteration
                update_plot(1)

                # Update plot on slider click
                slider.on_changed(update_plot)

                # Show plot
                plt.show()

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
