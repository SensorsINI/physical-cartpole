from multiprocessing.connection import Listener
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
matplotlib.use('TkAgg')
from datetime import datetime
import pandas as pd
import seaborn as sns
sns.set()

LIVE_PLOT_KEEPSAMPLES = 100

address = ('0.0.0.0', 6000)     # Listen on all available interfaces
listener = Listener(address)

connection = listener.accept()
print(f'Connected to: {listener.last_accepted}')

# Initialize variables
data = []
header = None

fig, axs = plt.subplots(5, 2, figsize=(16,9), gridspec_kw={'width_ratios': [3, 1]})
fig.subplots_adjust(hspace=0.8)
fig.canvas.manager.set_window_title('Live Plot')

received = 0

def animate(i):
    global data, fig, axs, received, header

    # receive data from socket
    while connection.poll(0.01):
        buffer = connection.recv()

        if isinstance(buffer, list) and isinstance(buffer[0], str):  # Header received
            header = buffer
            print(f'Header received: {header}')
            continue
        elif isinstance(buffer, np.ndarray):  # Data row received
            data.append(buffer)
            received += 1
            data = data[-LIVE_PLOT_KEEPSAMPLES:]
        elif buffer == 'reset':
            data = []
            print('\nLive Plot Reset\n\n\n\n')
        elif buffer == 'save' and header is not None:
            filepath = 'LivePlot' + str(
                datetime.now().strftime('_%Y-%m-%d_%H-%M-%S'))
            df = pd.DataFrame(data, columns=header)
            df.to_csv(filepath + '.csv', index=False)
            plt.savefig(filepath + '.pdf')
            print(f'\nLive Plot saved: {filepath}.pdf')
            print(f'Live Data saved: {filepath}.csv\n\n\n\n')
        elif isinstance(buffer, str) and buffer == 'complete':  # End of data
            print('All data received.')
            df = pd.DataFrame(data, columns=header)
            print(df)
            # Optionally process the DataFrame here


        if received >= 10:
            received = 0
            # Your plotting logic here, for example:
            if len(data) > 0:
                df = pd.DataFrame(data, columns=header)
                time = df.index
                colors = plt.rcParams["axes.prop_cycle"]()

                for i in range(len(header)):
                    data_row = df.iloc[:, i]
                    color = next(colors)["color"]

                    # Timeline
                    if i < 5:
                        axs[i, 0].clear()
                        axs[i, 0].set_title(f"Min={data_row.min():.3f}, Max={data_row.max():.3f}, Mean={data_row.mean():.3f}, Std={data_row.std():.5f}, N={data_row.size}", size=8)
                        axs[i, 0].plot(time, data_row, label=header[i], marker='.', color=color, markersize=3, linewidth=0.2)
                        axs[i, 0].legend(loc='upper right')
                        axs[i, 0].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)

                    # Histogram
                    if i < 5:
                        axs[i, 1].clear()
                        axs[i, 1].hist(data_row, bins=50, label=header[i], color=color)
                        axs[i, 1].set_ylabel('Occurrences')
                        axs[i, 1].set_title(header[i])
                        axs[i, 1].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)

ani = animation.FuncAnimation(fig, animate, interval=200)
plt.show()
print('Finished')
