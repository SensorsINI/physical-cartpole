from multiprocessing.connection import Listener
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
matplotlib.use('TkAgg')

address = ('localhost', 6000)     # family is deduced to be 'AF_INET'
listener = Listener(address)

print('Listening ...')
connection = listener.accept()
print(f'Connected to: {listener.last_accepted}')


fig = plt.figure('Live Plot')
fig.subplots_adjust(hspace=0.5)
data = np.zeros((0,4))
labels = ['angle_raw', 'angleD_raw']
axs = []
for i in range(2):
    axs.append(fig.add_subplot(2, 1, i+1))

def animate(i):
    global data, fig, axs, labels

    # receive data from socket
    while connection.poll(0.001):
        buffer = connection.recv()
        if isinstance(buffer, np.ndarray):
            data = np.append(data, buffer, axis=0)
            data = data[-5000:]

    # plot received data
    if axs is not None:
        colors = plt.rcParams["axes.prop_cycle"]()
        for i in range(2):
            color = next(colors)["color"]
            axs[i].clear()
            axs[i].set_title(f"Min: {data[:, i+1].min():.3f}, Max: {data[:, i+1].max():.3f}, Mean: {data[:, i+1].mean():.3f}, Std: {data[:, i+1].std():.3f}", size=8)
            axs[i].plot(data[:, 0], data[:, i+1], label=labels[i], marker='.', color=color)
            axs[i].legend(loc='upper right')
            axs[i].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)
            for b in data[data[:, 3]>0,0]:
                axs[i].axvline(x=b, color='red', linestyle='--', alpha=0.7)


ani = animation.FuncAnimation(fig, animate, interval=500)
plt.show()
fig.close()
print('Finished')