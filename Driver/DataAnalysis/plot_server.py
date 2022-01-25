from multiprocessing.connection import Listener
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
matplotlib.use('TkAgg')
from globals import *
from datetime import datetime
import math
import seaborn as sns
sns.set()

address = ('localhost', 6000)     # family is deduced to be 'AF_INET'
listener = Listener(address)

connection = listener.accept()
print(f'Connected to: {listener.last_accepted}')

labels_raw = ['angle_raw', 'angleD_raw', 'position_raw', 'positionD', 'actualMotorCmd']
labels_metric = ['angle [rad]', 'angleD [rad/s]', 'position [cm]', 'positionD [cm/s]', 'Q ∈ [-1,1]']
data = np.zeros((0, 7))

metric = LIVE_PLOT_UNITS

fig, axs = plt.subplots(5, 2, figsize=(16,9), gridspec_kw={'width_ratios': [3, 1]})
fig.subplots_adjust(hspace=0.5)
fig.canvas.manager.set_window_title('Live Plot')

received = 0

def animate(i):
    global data, fig, axs, labels, received, metric

    # receive data from socket
    while connection.poll(0.01):
        buffer = connection.recv()

        if isinstance(buffer, str):
            if buffer == 'raw':
                metric = 'raw'
            elif buffer == 'metric':
                metric = 'metric'
            elif buffer == 'reset':
                data = np.zeros((0, 7))
                print('\nLive Plot Reset\n\n\n\n')
            elif buffer == 'save':
                filepath = PATH_TO_EXPERIMENT_RECORDINGS + CONTROLLER_NAME + str(datetime.now().strftime('_%Y-%m-%d_%H-%M-%S'))
                np.savetxt(filepath+'.csv', data, delimiter=",", header=",".join(['firmware time']+(labels_raw if metric == 'raw' else labels_metric)+['frozen']))
                plt.savefig(filepath+'.pdf')
                print(f'\nLive Plot saved: {filepath}.pdf')
                print(f'Live Data saved: {filepath}.csv\n\n\n\n')

        if isinstance(buffer, np.ndarray):
            if LIVE_PLOT_ZERO_ANGLE_DOWN and metric == 'metric':
                angle = buffer[:, 1] + math.pi
                buffer[:, 1] = np.arctan2(np.sin(angle), np.cos(angle))
            data = np.append(data, buffer, axis=0)
            data = data[-LIVE_PLOT_KEEPSAMPLES:]

        received += 1

    if received >= 10:
        received = 0
        received = False
        time = data[:, 0]
        colors = plt.rcParams["axes.prop_cycle"]()

        for i in range(5):
            data_row = data[:, i+1]
            label = labels_raw[i] if metric == 'raw' else labels_metric[i]
            color = next(colors)["color"]

            # Timeline
            if i in LIVE_PLOT_TIMELINES:
                axs[i, 0].clear()
                axs[i, 0].set_title(f"Min={data_row.min():.3f}, Max={data_row.max():.3f}, Mean={data_row.mean():.3f}, Std={data_row.std():.5f}, N={data_row.size}", size=8)
                axs[i, 0].plot(time, data_row, label=label, marker='.', color=color, markersize=3, linewidth=0.2)
                axs[i, 0].legend(loc='upper right')
                axs[i, 0].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)
                for b in data[data[:, -1] > 0, 0]:
                    axs[i, 0].axvline(x=b, color='red', linestyle='--', alpha=0.7)
                    labels = [item.get_text() for item in axs[i, 0].get_xticklabels()]
                    labels[1] = 'Testing'

            # Histogramm
            if i in LIVE_PLOT_HISTOGRAMMS:
                axs[i, 1].clear()
                axs[i, 1].hist(data_row, bins=50, label=label, color=color)
                axs[i, 1].set_ylabel('occurences')
                axs[i, 1].set_xlabel(label)
                axs[i, 1].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)


ani = animation.FuncAnimation(fig, animate, interval=50)
plt.show()
print('Finished')