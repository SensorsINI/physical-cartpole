from multiprocessing.connection import Listener
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

address = ('localhost', 6000)     # family is deduced to be 'AF_INET'
listener = Listener(address)

print('Listening ...')
connection = listener.accept()
print(f'Connected to: {listener.last_accepted}')


fig = plt.figure('Live Plot')
fig.subplots_adjust(hspace=0.5)
data = np.zeros((0,0))
labels = ['angle_raw', 'angle', 'position']

def animate(i):
    global data, fig, axs, labels

    # receive data from socket
    while connection.poll(0.001):
        buffer = connection.recv()
        if isinstance(buffer, np.ndarray):
            if buffer.shape[1] != data.shape[1]:
                data = buffer
                axs = []
                for i in range(buffer.shape[1]):
                    axs.append(fig.add_subplot(buffer.shape[1], 1, i+1))
            else:
                data = np.append(data, buffer, axis=0)
                data = data[-5000:]

    # plot received data
    if axs is not None:
        colors = plt.rcParams["axes.prop_cycle"]()
        for i in range(data.shape[1]):
            color = next(colors)["color"]
            axs[i].clear()
            axs[i].set_title(f"Min: {data[:, i].min():.3f}, Max: {data[:, i].max():.3f}, Mean: {data[:, i].mean():.3f}, Std: {data[:, i].std():.3f}", size=8)
            axs[i].plot(data[:, i], label=labels[i], marker='.', color=color)
            axs[i].legend(loc='upper right')
            axs[i].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)

ani = animation.FuncAnimation(fig, animate, interval=50)
plt.show()
fig.close()
print('Finished')