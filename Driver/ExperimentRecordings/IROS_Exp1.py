import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

dataset_nni_hls = './nc_zynq_v1/iros24-ex1-experiment-4.csv'
dataset_nni_pc = './nc_pc_v1/iros24-ex1-experiment-0.csv'
# dataset_nni_pc = './nc_zynq_v1/iros24-ex1-experiment-2.csv' # hls, just to compare

def get_data(dataset):
    df_raw = pd.read_csv(dataset, comment='#')

    time = df_raw['time'].to_numpy()
    time = time - time[0]
    position = df_raw['position'].to_numpy()
    target_position = df_raw['target_position'].to_numpy()
    angle = df_raw['angle'].to_numpy()
    return time, position, target_position, angle


time_nni_hls, position_nni_hls, target_position_nni_hls, angle_nni_hls = get_data(dataset_nni_hls)
time_nni_pc, position_nni_pc, target_position_nni_pc, angle_nni_pc = get_data(dataset_nni_pc)

# Creating a figure with 2 subplots, arranged vertically (2 rows, 1 column)
fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # Adjust figsize as needed

# First subplot for position
axs[0].plot(time_nni_hls, position_nni_hls*100.0, label='position nc zynq')
axs[0].plot(time_nni_pc, position_nni_pc*100.0, label='position nc pc')
axs[0].plot(time_nni_hls, target_position_nni_hls*100.0, label='target_position')
axs[0].plot(time_nni_pc, target_position_nni_pc*100.0, label='target_position_pc')
axs[0].set_xlabel('time [s]')
axs[0].set_ylabel('position [cm]')
axs[0].legend()
axs[0].grid()

# Second subplot for angle
axs[1].plot(time_nni_hls, angle_nni_hls*180.0/np.pi, label='angle nc zynq')
axs[1].plot(time_nni_pc, angle_nni_pc*180.0/np.pi, label='angle nc pc')
axs[1].set_xlabel('time [s]')
axs[1].set_ylabel('angle [deg]')
axs[1].legend()
axs[1].grid()

plt.tight_layout()  # Adjust subplots to fit into the figure area.
plt.show()
plt.grid()
plt.show()