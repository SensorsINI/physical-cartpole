import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

FILE_NAME = 'ICRA_NN.csv'
FILE_NAME_2 = 'ICRA_NN.csv'
# FILE_NAME = None
df = pd.read_csv(FILE_NAME, comment='#')
time = df['time'].to_numpy()
time = time-time[0]
idx = np.where(time < 10.0)
time = time[idx]
angle = df['angle'].to_numpy()[idx]
position = df['position'].to_numpy()[idx] * 100.0  # Change to cm
target_position = df['target_position'].to_numpy()[idx] * 100.0  # Change to cm
control = df['Q'].to_numpy()[idx]

# df = pd.read_csv(FILE_NAME_2, comment='#')
# time_n = df['time'].to_numpy()
# time_n = time-time[0]
# idx = np.where(time < 10.0)
# time_n = time[idx]
# angle_n = df['angle'].to_numpy()[idx]
# position_n = df['position'].to_numpy()[idx] * 100.0  # Change to cm
# target_position_n = df['target_position'].to_numpy()[idx] * 100.0  # Change to cm
# control_n = df['Q'].to_numpy()[idx]


fig, axs = plt.subplots(3)

axs[0].set_title(FILE_NAME)
axs[0].plot(time, position)
axs[0].plot(time, target_position, '--', color='gray')

axs[0].set(ylabel='Position [cm]')
axs[1].plot(time, angle, label='ODE')
axs[1].set(ylabel='Angle [rad]')
axs[2].plot(time, control)
axs[2].set(ylabel="Control $u_k$")
axs[2].set(xlabel="Time [s]")

# axs[0].plot(time_n, position_n)
# # axs[0].plot(time_n, target_position_n, '--', color='gray')
# axs[1].plot(time_n, angle_n, label='neural')
# axs[2].plot(time_n, control_n)
# axs[1].legend()

plt.show()

fig.savefig(FILE_NAME[:-4]+'.png')