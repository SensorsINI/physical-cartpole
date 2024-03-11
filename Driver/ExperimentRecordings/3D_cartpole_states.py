import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from IROS_Exp1 import get_data, break_line_on_jump

# Load the datasets
dataset_nni_hls = './nc_zynq_v1/iros24-ex1-experiment-4.csv'

l = 0.395  # length of the pole, m
track_boundaries = 19.8  # boundaries of the track, cm

time, position, target_position, angle = get_data(dataset_nni_hls)

time4target, target_position = break_line_on_jump(time, target_position, threshold=0.01)

angle_sin = np.sin(angle)
angle_cos = np.cos(angle)

boundary_plus = np.full_like(time, track_boundaries)  # Create an array filled with 19.8
boundary_minus = np.full_like(time, -track_boundaries)  # Create an array filled with -19.8

print(plt.style.available)
# plt.style.use('seaborn-v0_8-poster')



fig = plt.figure(figsize=(14, 8))
ax = fig.add_subplot(111, projection='3d')

# Set larger linewidth and smooth lines, similar to Plotly
ax.plot((-angle_sin * l + position) * 100.0, time, (angle_cos * l) * 100.0, label="Pole's Tip", color='blue', linewidth=4, alpha=0.9)

ax.plot(position * 100.0, time, zs=0, zdir='z', label='Cart', color='red', linewidth=4, alpha=0.9)

ax.plot(target_position * 100.0, time4target, zs=0, zdir='z', label='Target Position', color='brown', linewidth=6, alpha=0.9)

ax.plot(boundary_plus, time, zs=0, zdir='z', label='Track Boundaries', linestyle='--', color='black', linewidth=2, alpha=0.7)
ax.plot(boundary_minus, time, zs=0, zdir='z', linestyle='--', color='black', linewidth=2, alpha=0.7)

# Add labels and title with a bigger font size
ax.set_xlabel('Position [cm]', fontsize=16, labelpad=20)
ax.set_ylabel('Time [s]', fontsize=16, labelpad=20)
ax.set_zlabel('Height [cm]', fontsize=16, labelpad=20)
plt.title('3D Plot of Pole Dynamics', fontsize=20)

# Legend with large font size and without the frame
ax.legend(fontsize=14, frameon=False)

# Removing the grid to mimic Plotly's default look
ax.grid(False)
# Set pane colors to white for a cleaner look
ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False
# Remove pane lines
ax.xaxis.pane.set_edgecolor('white')
ax.yaxis.pane.set_edgecolor('white')
ax.zaxis.pane.set_edgecolor('white')

# Set the tick parameters for the axes
ax.tick_params(axis='both', which='major', labelsize=14)

# Optionally, you can disable the offset on the axes if you want to mimic Plotly's style further
ax.get_xaxis().get_major_formatter().set_useOffset(False)
ax.get_yaxis().get_major_formatter().set_useOffset(False)
ax.get_zaxis().get_major_formatter().set_useOffset(False)

plt.show()