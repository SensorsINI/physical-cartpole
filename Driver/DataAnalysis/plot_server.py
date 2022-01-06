from multiprocessing.connection import Listener
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
matplotlib.use('TkAgg')

address = ('localhost', 6000)     # family is deduced to be 'AF_INET'
listener = Listener(address)

connection = listener.accept()
print(f'Connected to: {listener.last_accepted}')

labels_raw = ['angle_raw', 'angleD_raw', 'position_raw', 'positionD', 'actualMotorCmd']
labels_metric = ['angle', 'angleD', 'position', 'positionD', 'Q']
data = np.zeros((0, 7))

metric = 'raw'

fig, axs = plt.subplots(5, 2, figsize=(16,9), gridspec_kw={'width_ratios': [3, 1]})
fig.subplots_adjust(hspace=0.5)
fig.canvas.manager.set_window_title('Live Plot')


received = False

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

        if isinstance(buffer, np.ndarray):
            data = np.append(data, buffer, axis=0)
            data = data[-5000:]

        received = True

    if received:
        received = False
        time = data[:, 0]
        colors = plt.rcParams["axes.prop_cycle"]()

        for i in range(data.shape[1]-2):
            _data = data[:, i+1]
            label = labels_raw[i] if metric == 'raw' else labels_metric[i]
            color = next(colors)["color"]

            # Timeline
            axs[i, 0].clear()
            axs[i, 0].set_title(f"Min: {_data.min():.3f}, Max: {_data.max():.3f}, Mean: {_data.mean():.3f}, Std: {_data.std():.5f}", size=8)
            axs[i, 0].plot(time, _data, label=label, marker='.', color=color)
            axs[i, 0].legend(loc='upper right')
            axs[i, 0].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)
            for b in data[data[:, -1] > 0, 0]:
                axs[i, 0].axvline(x=b, color='red', linestyle='--', alpha=0.7)

            # Histogramm
            if label == 'angleD':
                axs[i, 1].hist(_data, bins=50, label=label, color=color)
                axs[i, 1].set_ylabel('occurences')
                axs[i, 1].set_xlabel(label)
                axs[i, 1].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)


ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()
print('Finished')