import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from IROS_Exp1 import get_data, break_line_on_jump

# Load the datasets
dataset_nni_hls = 'hardware_experiment_recording.csv'

l = 0.395  # length of the pole, m
track_boundaries = 19.8  # boundaries of the track, cm

time, position, target_position, angle = get_data(dataset_nni_hls)

time4angle, angle, position4angle = break_line_on_jump(time, angle, threshold=0.03, z=position)
# time4angle, angle, position4angle = time, angle, position
time4target, target_position = break_line_on_jump(time, target_position, threshold=0.01)

angle_sin = np.sin(angle)
angle_cos = np.cos(angle)

boundary_plus = np.full_like(time, track_boundaries)  # Create an array filled with 19.8
boundary_minus = np.full_like(time, -track_boundaries)  # Create an array filled with -19.8


fig = plt.figure()  # Adjust figure size
ax = fig.add_subplot(111, projection='3d')

# 3D curve
ax.plot((-angle_sin * l + position4angle) * 100.0, time4angle, (angle_cos * l) * 100.0, label="Pole's Tip", color='b')

# Adding 2D line on the same plot. We use one of the axes for position and another for time, with a constant value for the third axis.
# Adjust the 'zs' parameter to control the position's depth, making it appear as a 2D line.
ax.plot(position * 100.0, time, zs=0, zdir='z', label='Cart', color='r')

ax.plot(target_position * 100.0, time4target, zs=0, zdir='z', label='Target Position', color='brown')

ax.plot(boundary_plus, time, zs=0, zdir='z', label='Track Boundaries', linestyle='--', color='black')
ax.plot(boundary_minus, time, zs=0, zdir='z', linestyle='--', color='black')


# Set axis labels with increased labelpad for more distance
ax.set_xlabel('Position [cm]', labelpad=5)
ax.set_ylabel('Time [s]', labelpad=15)
ax.set_zlabel('Height [cm]', labelpad=5)

# plt.legend(loc='lower right')

ax.set_box_aspect((1, 3, 1))
plt.tight_layout()
plt.show()
