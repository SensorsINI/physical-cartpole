import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import os
import numpy as np

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "1"
from SI_Toolkit.TF.TF_Functions.predictor_autoregressive_tf import predictor_autoregressive_tf
import tensorflow as tf
from tqdm import tqdm
from matplotlib import use
import subprocess
from matplotlib.widgets import Slider
import glob

sns.set()
sns.color_palette()


def plot_predictive_performance(file, states, predictors, offsets=[0], title='', horizon=50, figsize=(16, 12), fontsize='medium', slider=False, plot_frozen=False, plot_fitted=False):
    units = {
        'angle': r'$\theta$ in rad',
        'angle_raw': 'raw ADC',
        'angleD': r'$\dot{\theta}$ in rad/s',
        'angleD_raw': 'raw ADC',
        'position': 'x in m',
        'positionD': r'$\dot{x}$ in m/s',
        'Q': 'Q ∈ [-1,1]'
    }
    limits = [None, None, None, None, None]
    trajectory = pd.read_csv(file, comment='#')
    time = trajectory['time'] - trajectory['time'][0]
    experiment_length = trajectory.shape[0]
    horizon = 50
    colors = ['red', 'green', 'blue', 'orange', 'magenta', 'yellow', 'cyan']

    # Calculate Predictions
    predictions = np.zeros((len(predictors.keys()), experiment_length, horizon + 1, 6))
    for i, name in enumerate(predictors):
        predictor = predictors[name]
        Q = np.pad(trajectory['Q'].to_numpy(), pad_width=(0, horizon))
        s = trajectory[['angle', 'angleD', 'angle_cos', 'angle_sin', 'position', 'positionD']].to_numpy()

        # Get predictions for whole trajectory
        for timestep in tqdm(range(experiment_length)):
            s_current = tf.convert_to_tensor(s[timestep, :], dtype=tf.float32)
            Q_current = tf.convert_to_tensor(Q[np.newaxis, timestep:timestep + horizon], dtype=tf.float32)
            predictions[i, timestep, 0] = s[timestep, :]
            predictions[i, timestep, 1:] = predictor.predict_tf(s_current, Q_current).numpy()
            predictor.update_internal_state_tf(tf.convert_to_tensor(Q_current[:, 0], dtype=tf.float32))

    if slider:
        use('TkAgg')
        fig, axs = plt.subplots(nrows=len(states), ncols=1, sharex=True, gridspec_kw={"bottom": 0.15, "left": 0.1, "right": 0.84, "top": 0.95}, figsize=figsize)
        slider_axis = plt.axes([0.15, 0.02, 0.7, 0.03])
        slider = Slider(slider_axis, "timestep", 1, experiment_length, valinit=offsets[0], valstep=1)
    else:
        fig, axs = plt.subplots(len(states), 1, figsize=figsize, sharex=True)

    def plot_all(offsets):
        if not isinstance(offsets, list):
            offsets = [offsets]

        # Plot Trajectory
        for j, state in enumerate(states):
            axs[j].clear()
            axs[j].plot(time, trajectory[state], label='Physical Trajectory', marker='.', markersize=2, linewidth=1, linestyle='--', color='black', alpha=0.5)
            if state == 'angle_raw' and 'angle_raw_sensor' in trajectory:
                axs[j].plot(time, trajectory['angle_raw_sensor'], label='Sensor', marker='.', markersize=2, linewidth=1, linestyle='--', color='blue', alpha=0.5)
            if state == 'angleD_raw' and 'angleD_raw_sensor' in trajectory:
                axs[j].plot(time, trajectory['angleD_raw_sensor'], label='Sensor', marker='.', markersize=2, linewidth=1, linestyle='--', color='blue', alpha=0.5)
            if state == 'angleD_raw' and 'angleD_fitted' in trajectory:
                axs[j].plot(time, trajectory['angleD_fitted'], label='Fitted', marker='.', markersize=2, linewidth=1, linestyle='--', color='green', alpha=0.5)
            axs[j].set_title(units[state], fontsize=fontsize)
            axs[j].tick_params(axis='both', which='major', labelsize=fontsize)
            axs[j].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5, alpha=0.5)
            if state != 'Q':
                for offset in offsets:
                    axs[j].plot(time[offset], trajectory[state][offset], marker='o', markersize=5, color='black', alpha=0.5)

        # Plot Frozen
        if plot_frozen and 'frozen' in trajectory:
            frozen = trajectory['frozen'].to_numpy().nonzero()[0]
            if len(frozen) > 0:
                for j, state in enumerate(states):
                    for x in np.nditer(frozen):
                        axs[j].axvline(x=time[x], color='red', linestyle='-', alpha=0.7)

        # Plot Fitted
        if plot_fitted and 'fitted' in trajectory:
            fitted = trajectory['fitted'].to_numpy().nonzero()[0]
            if len(fitted) > 0:
                for j, state in enumerate(states):
                    for x in np.nditer(fitted):
                        axs[j].axvline(x=time[x], color='green', linestyle='--', alpha=0.7)

        # Plot Predictions
        indexes = {'angle': 0, 'angleD': 1, 'position': 4, 'positionD': 5}
        for k, offset in enumerate(offsets):
            for i, name in enumerate(predictors):
                for j, state in enumerate(states):
                    if state in indexes:
                        axs[j].plot(time[offset:offset + horizon + 1], predictions[i, offset, :, indexes[state]], label=name if k == 0 else None, marker='.', markersize=2, linewidth=1, linestyle='--', color=colors[i])

        # Add Legend
        for j in range(len(states)):
            axs[j].legend(loc='upper right', fontsize=fontsize)

    plot_all(offsets)
    if slider:
        slider.on_changed(plot_all)
    else:
        fig.tight_layout()

    plt.xlabel('time in s', fontsize=fontsize)
    pdf = 'ExperimentRecordings/Plots/Predictive Performance - ' + title + '.pdf'
    plt.savefig(pdf)
    plt.show()
    if not slider:
        subprocess.call(['evince', pdf])


if __name__ == "__main__":
    horizon = 50

    # plot_predictive_performance(
    #     title='Varying Stepsize',
    #     file = 'ExperimentRecordings/20ms Trajectories/CP_mppi-tf_2022-02-11_16-47-00 20ms swingup.csv',
    #     states = ['angle', 'angleD', 'position', 'positionD'],
    #     predictors = {
    #         'Euler (h=20ms)': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='EulerTF', intermediate_steps=1),
    #         'Euler (h=10ms)': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='EulerTF', intermediate_steps=5),
    #         'Euler (h=2ms)': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='EulerTF', intermediate_steps=10),
    #         'Euler (h=1ms)': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='EulerTF', intermediate_steps=20),
    #     },
    #     offsets = [51, 150],
    #     figsize=(8,12),
    #     fontsize='small'
    # )

    # plot_predictive_performance(
    #     title='Varying k',
    #     file = 'ExperimentRecordings/20ms Trajectories/CP_PID_2022-01-22_01-34-16 20ms swinging short.csv',
    #     states = ['angle', 'angleD'],
    #     predictors = {
    #         'Euler (k=4/3)': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='EulerTF', intermediate_steps=10, k=4/3),
    #         'Euler (k=1/3)': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='EulerTF', intermediate_steps=10, k=1/3),
    #     },
    #     offsets = [51, 150],
    #     figsize=(8,6)
    # )
    last_files = glob.glob('ExperimentRecordings/*.csv')
    last_files.sort(key=os.path.getctime, reverse=True)

    plot_predictive_performance(
        title='Euler & RNN',
        # file = 'ExperimentRecordings/CP_mppi-tf-RNN_2022-02-18_01-25-26 Unsuccesful Swingup.csv',
        # file = 'ExperimentRecordings/CP_mppi-tf-RNN_2022-02-18_01-25-26 Unsuccesful Swingup.csv',
        file=last_files[0],
        # file='ExperimentRecordings/Swingups for Training V2/Experiment-0.csv',
        states=['angle', 'angleD', 'position', 'positionD', 'Q'],
        predictors={
            'Euler': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='EulerTF', intermediate_steps=10, k=1 / 3),
            # 'Pretrained RNN': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='Pretrained-RNN-1/GRU-6IN-32H1-32H2-5OUT-0'),
            # 'RNN with Simulation Data (20 Epochs) [GRU-1]': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='PhysicalData-1/GRU-6IN-32H1-32H2-5OUT-1'),
            'RNN with Physical V1 (20 Epochs) [GRU-4]': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='PhysicalData-1/GRU-6IN-32H1-32H2-5OUT-4'),
            # 'RNN with Simulation (20 Epochs) + Physical V1 (2 Epochs) [GRU-0]': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='PhysicalData-1/GRU-6IN-32H1-32H2-5OUT-0'),
            # 'RNN with Simulation (20 Epochs) + Physical V1 (20 Epochs) [GRU-3]': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='PhysicalData-1/GRU-6IN-32H1-32H2-5OUT-3'),
            # 'RNN with Simulation + Physical V1 (20 Epochs) + Physical V2 (10 epochs) [GRU-5]': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='PhysicalData-1/GRU-6IN-32H1-32H2-5OUT-5'),
            # 'RNN with Simulation + Physical V1 (20 Epochs) + Physical V2 (20 epochs) [GRU-6]': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='PhysicalData-1/GRU-6IN-32H1-32H2-5OUT-6'),
            'RNN with Physical V2 (20 Epochs) [GRU-7]': predictor_autoregressive_tf(horizon=horizon, batch_size=1, net_name='PhysicalData-1/GRU-6IN-32H1-32H2-5OUT-7'),
        },
        offsets=[237],
        slider=True,
        plot_frozen=True,
        plot_fitted=True
    )
