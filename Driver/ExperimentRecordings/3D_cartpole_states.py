import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from IROS_Exp1 import get_data, break_line_on_jump

# Load the datasets
dataset_nni_hls = './nc_zynq_v1/iros24-ex1-experiment-4.csv'

l = 0.395  # length of the pole, m
track_boundaries = 19.8  # boundaries of the track, cm

time, position, target_position, angle = get_data(dataset_nni_hls)

angle_sin = np.sin(angle)
angle_cos = np.cos(angle)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 3D curve
ax.plot((-angle_sin * l + position) * 100.0, time, (angle_cos * l) * 100.0, label="Pole's Tip", color='b')

# Adding 2D line on the same plot. We use one of the axes for position and another for time, with a constant value for the third axis.
# Adjust the 'zs' parameter to control the position's depth, making it appear as a 2D line.
ax.plot(position * 100.0, time, zs=0, zdir='z', label='Cart', color='r')

time4target, target_position = break_line_on_jump(time, target_position, threshold=0.01)

ax.plot(target_position * 100.0, time4target, zs=0, zdir='z', label='Target Position', color='brown')

boundary_plus = np.full_like(time, track_boundaries)  # Create an array filled with 19.8
boundary_minus = np.full_like(time, -track_boundaries)  # Create an array filled with -19.8

ax.plot(boundary_plus, time, zs=0, zdir='z', label='Track Boundaries', linestyle='--', color='black')
ax.plot(boundary_minus, time, zs=0, zdir='z', linestyle='--', color='black')


ax.set_xlabel('Position [cm]')
ax.set_ylabel('Time [s]')
ax.set_zlabel('Height [cm]')

plt.legend()
plt.show()
