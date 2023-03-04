import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

FILE_NAME = 'ICRA_ODE_2.csv'
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

df = pd.read_csv(FILE_NAME_2, comment='#')
time_n = df['time'].to_numpy()
time_n = time-time[0]
idx = np.where(time < 10.0)
time_n = time[idx]
angle_n = df['angle'].to_numpy()[idx]
position_n = df['position'].to_numpy()[idx] * 100.0  # Change to cm
target_position_n = df['target_position'].to_numpy()[idx] * 100.0  # Change to cm
control_n = df['Q'].to_numpy()[idx]


fig, axs = plt.subplots(3)

y_labelpad = 4
axes_order = {'position': 0, 'angle': 1, 'control': 2}

# axs[0].set_title(FILE_NAME)
axs[axes_order['position']].plot(time, position)
axs[axes_order['position']].plot(time, target_position, '--', color='gray', label='Target position')

axs[axes_order['position']].set_ylabel('Position [cm]', labelpad=y_labelpad+3.0)
axs[axes_order['angle']].plot(time, angle, label='RPGD with ODE model')
axs[axes_order['angle']].set_ylabel('Angle [rad]', labelpad=y_labelpad)
axs[axes_order['control']].plot(time, control)
axs[axes_order['control']].set_ylabel("Control normed", labelpad=y_labelpad+8.0)
axs[axes_order['control']].set(xlabel="Time [s]")

axs[axes_order['position']].plot(time_n, position_n)
axs[axes_order['position']].legend(handletextpad=0.3, borderaxespad=0.3, loc='lower right')
# axs[0].plot(time_n, target_position_n, '--', color='gray')
axs[axes_order['angle']].plot(time_n, angle_n, label='RPGD with neural network model')
axs[axes_order['control']].plot(time_n, control_n)
axs[axes_order['angle']].legend(handletextpad=0.3, borderaxespad=0.3, loc='lower right')
plt.tight_layout()
plt.show()

# fig.savefig(FILE_NAME[:-4]+'.png')
fig.savefig('Data-RPGD-tf'+'.png')