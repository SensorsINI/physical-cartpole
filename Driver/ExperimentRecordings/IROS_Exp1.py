import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

dataset_nni_hls = './nc_zynq_v1/iros24-ex1-experiment-4.csv'
dataset_nni_pc = './nc_pc_v1/iros24-ex1-experiment-1.csv'
# dataset_nni_pc = './nc_zynq_v1/iros24-ex1-experiment-2.csv' # hls, just to compare

axis_labels_fontsize = 12

def get_data(dataset):
    df_raw = pd.read_csv(dataset, comment='#')

    time = df_raw['time'].to_numpy()
    time = time - time[0]
    position = df_raw['position'].to_numpy()
    target_position = df_raw['target_position'].to_numpy()
    angle = df_raw['angle'].to_numpy()
    return time, position, target_position, angle


def break_line_on_angle_jump(time, angle):
    # Threshold for the difference
    threshold = 1.0  # radians

    # Containers for the modified data
    time_modified = []
    angle_modified = []

    # Loop through the data and insert np.nan where the difference exceeds the threshold
    for i in range(1, len(angle)):
        time_modified.append(time[i - 1])
        angle_modified.append(angle[i - 1])

        if np.abs(angle[i] - angle[i - 1]) > threshold:
            # Insert np.nan to break the line
            time_modified.append(np.nan)
            angle_modified.append(np.nan)

    # Don't forget to add the last point
    time_modified.append(time[-1])
    angle_modified.append(angle[-1])

    # Convert to numpy arrays
    time_modified = np.array(time_modified)
    angle_modified = np.array(angle_modified)

    return time_modified, angle_modified



# Color definitions for clarity and consistency
prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']

color_hls = colors[0]
color_pc = colors[1]
color_target = colors[3]


time_nni_hls, position_nni_hls, target_position_nni_hls, angle_nni_hls = get_data(dataset_nni_hls)
time_nni_pc, position_nni_pc, target_position_nni_pc, angle_nni_pc = get_data(dataset_nni_pc)

# Creating a figure with 2 subplots, arranged vertically (2 rows, 1 column)
fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # Adjust figsize as needed

# First subplot for position
axs[0].plot(time_nni_hls, position_nni_hls*100.0, label='position nc zynq')
axs[0].plot(time_nni_pc, position_nni_pc*100.0, label='position nc pc')
axs[0].plot(time_nni_hls, target_position_nni_hls*100.0, color=color_target, label='target_position')
# axs[0].plot(time_nni_pc, target_position_nni_pc*100.0, label='target_position_pc')
axs[0].set_xlabel('Time [s]', fontsize=axis_labels_fontsize)
axs[0].set_ylabel('Position [cm]', fontsize=axis_labels_fontsize)
axs[0].legend()
axs[0].grid()

angle_swing_up_nni_hls = angle_nni_hls[time_nni_hls < 8]
angle_stable_nni_hls = angle_nni_hls[time_nni_hls >= 8]
time_swing_up_nni_hls = time_nni_hls[time_nni_hls < 8]
time_stable_nni_hls = time_nni_hls[time_nni_hls >= 8]

time_swing_up_nni_hls, angle_swing_up_nni_hls = break_line_on_angle_jump(time_swing_up_nni_hls, angle_swing_up_nni_hls)

angle_swing_up_nni_pc = angle_nni_pc[time_nni_pc < 8]
angle_stable_nni_pc = angle_nni_pc[time_nni_pc >= 8]
time_swing_up_nni_pc = time_nni_pc[time_nni_pc < 8]
time_stable_nni_pc = time_nni_pc[time_nni_pc >= 8]

time_swing_up_nni_hls, angle_swing_up_nni_hls = break_line_on_angle_jump(time_swing_up_nni_hls, angle_swing_up_nni_hls)
time_swing_up_nni_pc, angle_swing_up_nni_pc = break_line_on_angle_jump(time_swing_up_nni_pc, angle_swing_up_nni_pc)

# Second subplot for angle with condition for time < 8.0
axs[1].plot(time_swing_up_nni_hls, angle_swing_up_nni_hls*180.0/np.pi, color=color_hls, label='angle nc zynq')
axs[1].plot(time_swing_up_nni_pc, angle_swing_up_nni_pc*180.0/np.pi, color=color_pc, label='angle nc pc')

# Use twinx() for angles where time >= 8.0
ax2 = axs[1].twinx()
ax2.plot(time_nni_hls[time_nni_hls >= 8], angle_nni_hls[time_nni_hls >= 8]*180.0/np.pi, color=color_hls, label='NC Zynq')
ax2.plot(time_nni_pc[time_nni_pc >= 8], angle_nni_pc[time_nni_pc >= 8]*180.0/np.pi, color=color_pc, label='NC PC')

# Set labels for the y-axes
axs[1].set_ylabel('Angle [deg] for t < 8s', fontsize=axis_labels_fontsize)
ax2.set_ylabel('Angle [deg] for t >= 8s', fontsize=axis_labels_fontsize)

# Since we use the same labels for matching datasets, we can just create one legend.
# No need to manually handle the labels and handles from both axes.
handles, labels = axs[1].get_legend_handles_labels()

# Optionally filter out duplicate labels/handles if you want to ensure the legend is concise
from collections import OrderedDict
by_label = OrderedDict(zip(labels, handles))  # Filters out duplicates

# Now use this deduplicated dictionary to create the legend
ax2.legend(by_label.values(), by_label.keys(), loc='upper right')

axs[1].set_xlabel('Time [s]', fontsize=axis_labels_fontsize)  # No change here

# Add vertical lines to indicate change of y axis
axs[1].axvline(x=8.0, linestyle='--', linewidth=3, color='#8B4513', label='x = 8.0')  # Hex code for a shade of dark brown

# Make sure 0 level is well alligened
max_abs_angle_hls = max(np.abs(angle_nni_hls[time_nni_hls < 8] * 180.0 / np.pi))
max_abs_angle_pc = max(np.abs(angle_nni_pc[time_nni_pc < 8] * 180.0 / np.pi))
max_y_left = max(max_abs_angle_hls, max_abs_angle_pc)

max_abs_angle_hls = max(np.abs(angle_nni_hls[time_nni_hls >= 8] * 180.0 / np.pi))
max_abs_angle_pc = max(np.abs(angle_nni_pc[time_nni_pc >= 8] * 180.0 / np.pi))
max_y_right = max(max_abs_angle_hls, max_abs_angle_pc)

factor = 1.1
axs[1].set_ylim(-factor*max_y_left, factor*max_y_left)
ax2.set_ylim(-factor*max_y_right, factor*max_y_right)

axs[1].xaxis.grid(True)

left_yticks = axs[1].get_yticks()
right_yticks = ax2.get_yticks()

axs[1].autoscale(False, axis="x")
ax2.autoscale(False, axis="x")
xlim = axs[1].get_xlim()
for ytick in left_yticks:
    # Draw lines manually for the right side, starting from x=8.0 to the end of the plot
    axs[1].hlines(y=ytick, xmin=xlim[0], xmax=8.0, color='grey', linestyle='-', linewidth=1.0, alpha=0.5)

# You might want to filter or select a subset of right_yticks if there are too many
xlim = ax2.get_xlim()
for ytick in right_yticks:
    # Draw lines manually for the right side, starting from x=8.0 to the end of the plot
    plt.hlines(y=ytick, xmin=8.0, xmax=xlim[1], color='grey', linestyle='-', linewidth=1.0, alpha=0.5)  # Adjust alpha, linestyle, and linewidth as needed



# Finish and display the plot
plt.tight_layout()  # Adjust subplots to fit into the figure area.
plt.show()


fig.savefig('IROS_Exp1.pdf')





