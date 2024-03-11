import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def get_data(dataset):
    df_raw = pd.read_csv(dataset, comment='#')

    time = df_raw['time'].to_numpy()
    time = time - time[0]
    position = df_raw['position'].to_numpy()
    target_position = df_raw['target_position'].to_numpy()
    angle = df_raw['angle'].to_numpy()
    return time, position, target_position, angle


def break_line_on_jump(x, y, threshold=1.0, z=None):

    # Containers for the modified data
    x_modified = []
    y_modified = []
    z_modified = []

    # Loop through the data and insert np.nan where the difference exceeds the threshold
    for i in range(1, len(y)):
        x_modified.append(x[i - 1])
        y_modified.append(y[i - 1])
        if z is not None:
            z_modified.append(z[i - 1])

        if np.abs(y[i] - y[i - 1]) > threshold:
            # Insert np.nan to break the line
            x_modified.append(np.nan)
            y_modified.append(np.nan)
            if z is not None:
                z_modified.append(np.nan)

    # Don't forget to add the last point
    x_modified.append(x[-1])
    y_modified.append(y[-1])
    if z is not None:
        z_modified.append(z[-1])

    # Convert to numpy arrays
    x_modified = np.array(x_modified)
    y_modified = np.array(y_modified)
    z_modified = np.array(z_modified)

    if z is not None:
        return x_modified, y_modified, z_modified
    else:
        return x_modified, y_modified


def get_colors():
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']

    A = colors[2]
    B = colors[1]
    color_target = colors[3]
    C = colors[0]
    D = colors[4]

    colors = (color_target, A, B, C, D)

    return colors


def plot_position(axs, datasets, labels, axis_labels_fontsize):

    colors = get_colors()

    for i, dataset in enumerate(datasets):
        time, position, _, _ = get_data(dataset)
        axs[0].plot(time, position * 100.0, color=colors[i + 1], label='Position ' + labels[i])


    # Plot target position using first dataset - assumed is that it is same everywhere
    time, _, target_position, _ = get_data(datasets[0])
    axs[0].plot(time, target_position * 100.0, color=colors[0], label='Target Position')


    axs[0].set_xlabel('Time [s]', fontsize=axis_labels_fontsize)
    axs[0].set_ylabel('Position [cm]', fontsize=axis_labels_fontsize)
    axs[0].legend()
    axs[0].grid()


def prepare_legend_angle(ax1, ax2):
    # Since we use the same labels for matching datasets, we can just create one legend.
    # No need to manually handle the labels and handles from both axes.
    handles, labels = ax1.get_legend_handles_labels()

    # Optionally filter out duplicate labels/handles if you want to ensure the legend is concise
    from collections import OrderedDict
    by_label = OrderedDict(zip(labels, handles))  # Filters out duplicates

    # Now use this deduplicated dictionary to create the legend
    ax2.legend(by_label.values(), by_label.keys(), loc='upper right')


def add_vertical_division_line(ax, threshold):
    ax.axvline(x=threshold, linestyle='--', linewidth=3, color='#8B4513', label=f'x = {threshold}')  # Hex code for a shade of dark brown


def grid_for_twinx(ax1, ax2, threshold):
    ax1.xaxis.grid(True)

    left_yticks = ax1.get_yticks()
    right_yticks = ax2.get_yticks()

    ax1.autoscale(False, axis="x")
    ax2.autoscale(False, axis="x")
    xlim = ax1.get_xlim()
    for ytick in left_yticks:
        # Draw lines manually for the right side, starting from x=8.0 to the end of the plot
        axs[1].hlines(y=ytick, xmin=xlim[0], xmax=threshold, color='grey', linestyle='-', linewidth=1.0, alpha=0.5)

    # You might want to filter or select a subset of right_yticks if there are too many
    xlim = ax2.get_xlim()
    for ytick in right_yticks:
        # Draw lines manually for the right side, starting from x=8.0 to the end of the plot
        plt.hlines(y=ytick, xmin=threshold, xmax=xlim[1], color='grey', linestyle='-', linewidth=1.0,
                   alpha=0.5)  # Adjust alpha, linestyle, and linewidth as needed


def center_0(axs, factor):
    ylim = np.array(axs.get_ylim())
    y_max = np.max(np.abs(ylim))
    axs.set_ylim(-factor * y_max, factor * y_max)

def get_swing_up_data(angle, time, threshold):
    angle_swing_up = angle[time < threshold]
    time_swing_up = time[time < threshold]
    return angle_swing_up, time_swing_up

def get_stable_data(angle, time, threshold):
    angle_stable = angle[time >= threshold]
    time_stable = time[time >= threshold]
    return angle_stable, time_stable


def plot_angle(axs, datasets, labels, axis_labels_fontsize):

    colors = get_colors()
    threshold = 8.0
    # First subplot for angle with condition for time < 8.0
    for i, dataset in enumerate(datasets):
        time, _, _, angle = get_data(dataset)
        angle_swing_up, time_swing_up = get_swing_up_data(angle, time, threshold)
        time_swing_up, angle_swing_up = break_line_on_jump(time_swing_up, angle_swing_up)
        axs[1].plot(time_swing_up, angle_swing_up*180.0/np.pi, color=colors[i + 1], label='Angle ' + labels[i])

    # Use twinx() for angles where time >= 8.0
    ax2 = axs[1].twinx()

    for i, dataset in enumerate(datasets):
        time, _, _, angle = get_data(dataset)
        angle_stable, time_stable = get_stable_data(angle, time, threshold)
        time_stable, angle_stable = break_line_on_jump(time_stable, angle_stable)
        ax2.plot(time_stable, angle_stable*180.0/np.pi, color=colors[i + 1], label='Angle ' + labels[i])

    # Set labels for the y-axes
    axs[1].set_ylabel(f'Angle [deg] for t < {int(threshold)}s', fontsize=axis_labels_fontsize)
    ax2.set_ylabel(f'Angle [deg] for t >= {int(threshold)}s', fontsize=axis_labels_fontsize)
    axs[1].set_xlabel('Time [s]', fontsize=axis_labels_fontsize)  # No change here


    prepare_legend_angle(axs[1], ax2)

    add_vertical_division_line(axs[1], threshold)

    factor = 1.1
    center_0(axs[1], factor)
    center_0(ax2, factor)

    grid_for_twinx(axs[1], ax2, threshold)

def plot_Q_fun(axs, datasets, labels, axis_labels_fontsize):
    colors = get_colors()

    for i, dataset in enumerate(datasets):
        df_raw = pd.read_csv(dataset, comment='#')
        time = df_raw['time'].to_numpy()
        time = time - time[0]
        Q = df_raw['Q'].to_numpy()
        axs[2].plot(time, Q, color=colors[i + 1], label='Q ' + labels[i])


    # Set labels for the y-axes
    axs[2].set_ylabel(f'Q', fontsize=axis_labels_fontsize)
    axs[2].set_xlabel('Time [s]', fontsize=axis_labels_fontsize)  # No change here


if __name__ == '__main__':
    # Load the datasets
    dataset_nni_hls = 'hardware_experiment_recording.csv'
    dataset_nni_pc = './nc_pc_v1/iros24-ex1-experiment-1.csv'
    # dataset_mpc_pc = './rpgd_pc_v1/iros24-ex1-experiment-2.csv'
    dataset_mpc_pc = 'iros24-ex1-experiment-1-3.csv'
    # dataset_nni_pc = './nc_zynq_v1/iros24-ex1-experiment-2.csv' # hls, just to compare

    datasets = [dataset_nni_hls, dataset_nni_pc, dataset_mpc_pc]
    labels = ['NC Zynq', 'NC PC', 'MPC PC']
    axis_labels_fontsize = 12
    plot_Q = True

    # Create a figure with 2 or 3 subplots
    if plot_Q:
        fig, axs = plt.subplots(3, 1, figsize=(15, 12))
    else:
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # Adjust figsize as needed

    # Plot the position data
    plot_position(axs, datasets, labels, axis_labels_fontsize)
    plot_angle(axs, datasets, labels, axis_labels_fontsize)
    if plot_Q:
        # Plot the Q data
        plot_Q_fun(axs, datasets, labels, axis_labels_fontsize)
    plt.tight_layout()  # Adjust subplots to fit into the figure area.
    # Show the plot
    plt.show()

    fig.savefig('IROS_Exp1.pdf')




