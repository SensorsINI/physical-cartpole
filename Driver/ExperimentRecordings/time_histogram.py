import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

target_control_latency = 50.0
dataset = 'CP_mpc_2024-01-06_16-34-45-10ms-STM-Flush.csv'

df_raw = pd.read_csv(dataset, comment='#')

try:
    latency_violations_vec = df_raw['latency_violations'].to_numpy()
    latency_violations = len(latency_violations_vec[latency_violations_vec == 1])
    samples = len(latency_violations_vec)
    print(f"total violations percentage {100.0 * latency_violations / samples} %")
    latency_violations_percentage = 100.0 * latency_violations / samples
    df = df_raw.loc[df_raw.latency_violations == 0, :]
except KeyError:
    latency_violations_percentage = -1000.0
    df = df_raw



latency = df['latency'].to_numpy()*1000.0
pythonLatency = df['pythonLatency'].to_numpy()*1000.0
controller_steptime = df['controller_steptime'].to_numpy()*1000.0

dt_controller = controller_steptime
python_prepost = pythonLatency-dt_controller
firmware = latency-controller_steptime


def annotate_with_mean_and_std(data, ax, units='ms', log_scale=True):
    plt.sca(ax)
    mean = np.mean(data)
    std = np.std(data)
    min_value = np.min(data)
    max_value = np.max(data)
    plt.axvline(mean, color='red', linestyle='dashed', linewidth=1)
    min_ylim, max_ylim = plt.ylim()
    log_max = np.log10(max_ylim)  # Adjust these based on your y-axis range

    if log_scale:
        y_positions = [10**(0.90 * log_max), 10**(0.85 * log_max), 10**(0.80 * log_max)]
    else:
        y_positions = [0.90 * max_ylim, 0.85 * max_ylim, 0.80 * max_ylim]

    plt.text(mean * 1.15, y_positions[0], f'Mean: {mean:.3f} {units}')
    plt.text(mean * 1.15, y_positions[1], f'Std: {std:.3f} {units}')
    plt.text(mean * 1.15, y_positions[2], f"Range: {min_value:.2f} {units} - {max_value:.2f} {units}")
    plt.xlabel(units)

fig, axs = plt.subplots(1, 4, tight_layout=True, figsize=(15, 6.0), sharex=True)


fig.suptitle(f"Latency from file {dataset}\n latency violations not included in histograms ({latency_violations_percentage:.2f}%)")

axs[0].hist(dt_controller, bins=100, color='blue', alpha=0.7)
axs[0].set_title('Controller')
annotate_with_mean_and_std(dt_controller, axs[0])
axs[0].set_yscale('log')

axs[1].hist(python_prepost, bins=100, color='red', alpha=0.7)
axs[1].set_title('Python pre- and postprocessing')
annotate_with_mean_and_std(python_prepost, axs[1])
axs[1].set_yscale('log')

axs[2].hist(firmware, bins=100, color='green', alpha=0.7)
axs[2].set_title('Firmware latency \n (without controller latency)')
annotate_with_mean_and_std(firmware, axs[2])
axs[2].set_yscale('log')

axs[3].hist(latency, bins=100, color='orange', alpha=0.7)
axs[3].set_title('Total latency')
annotate_with_mean_and_std(latency, axs[3], log_scale=False)
# axs[3].set_yscale('log')

plt.show()

#
# angle = df['angle'].to_numpy()
# position = df['position'].to_numpy()*100.0
# time = df['time'].to_numpy()
# time = time-time[0]
#
# fig2, axs = plt.subplots(2, 2, tight_layout=True, figsize=(10, 10.0))
# fig2.suptitle(f'Jitteriness from file {dataset}')
#
# plt.sca(axs[0, 0])
# plt.title('Angle')
# plt.plot(time, angle, color='red')
# plt.ylabel('rad')
# plt.xlabel('time [s]')
#
# axs[1, 0].hist(angle, bins=20, color='orange', alpha=0.7)
# annotate_with_mean_and_std(angle, axs[1, 0], units='rad')
#
#
# plt.sca(axs[0, 1])
# plt.title('Position')
# plt.plot(time, position, color='blue')
# plt.ylabel('position [cm]')
# plt.xlabel('time [s]')
#
# axs[1, 1].hist(position, bins=20, color='green', alpha=0.7)
# annotate_with_mean_and_std(position, axs[1, 1], units='cm')
#
# plt.show()

#
# x = np.sort(latency)
# y = np.cumsum(x)/np.sum(latency)
# plt.figure()
# plt.plot(x[3000:-3000],y[3000:-3000])
#
# plt.show()
