
import pandas as pd
import matplotlib.pyplot as plt

file_path = 'cartpole-2021-11-02-14-26-37.csv'

fontsize_labels = 14
fontsize_ticks = 12

data: pd.DataFrame = pd.read_csv(file_path, comment='#')  # skip comment lines starting with #

time = data['time']
latency = data['additional_latency']*1000.0
angle_squared = data['angle_squared']

fig, ax = plt.subplots(1, 1, figsize=(16, 9), sharex=True)
plt.title('Latency effect on PID stability')

angle_squared_plot = plt.plot(time, angle_squared, color='orange', label=' Squared angle (deg^2)')
ax.set_ylabel("Angle Squared (deg^2)", fontsize=fontsize_labels)
ax.set_xlabel("Time (s)", fontsize=fontsize_labels)

ax_latency = ax.twinx()
latency_plot = plt.plot(time, latency, label='Latency (ms)')
ax_latency.set_ylabel("Latency (ms)", fontsize=fontsize_labels)

lns = angle_squared_plot + latency_plot
labs = [l.get_label() for l in lns]
ax_latency.legend(lns, labs, fontsize=fontsize_labels)

plt.show()