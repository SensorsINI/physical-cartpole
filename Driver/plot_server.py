from multiprocessing.connection import Listener
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# import matplotlib
# matplotlib.use('TkAgg')
from datetime import datetime
import pandas as pd
import seaborn as sns

sns.set()

class LivePlotter:
    def __init__(self, fig=None, axs=None, address=('0.0.0.0', 6000), keep_samples=100):
        # Set up listener for incoming data
        self.listener = Listener(address)
        self.connection = self.listener.accept()
        print(f'Connected to: {self.listener.last_accepted}')

        self.data = []
        self.header = None
        self.received = 0
        self.keep_samples = keep_samples
        if fig is None or axs is None:
            self.fig, self.axs = plt.subplots(5, 2, figsize=(16, 9), gridspec_kw={'width_ratios': [3, 1]})
            self.fig.subplots_adjust(hspace=0.8)
            self.fig.canvas.manager.set_window_title('Live Plot')
        else:
            self.fig = fig
            self.axs = axs

    def animate(self, i):
        while self.connection.poll(0.01):
            buffer = self.connection.recv()
            self.process_buffer(buffer)

        if self.received >= 10:
            self.received = 0
            self.update_plots()

    def process_buffer(self, buffer):
        # Process incoming buffer based on its type
        if isinstance(buffer, list) and isinstance(buffer[0], str):
            self.header = buffer
            print(f'Header received: {self.header}')
        elif isinstance(buffer, np.ndarray):
            self.data.append(buffer)
            self.received += 1
            self.data = self.data[-self.keep_samples:]
        elif buffer == 'reset':
            self.data = []
            print('\nLive Plot Reset\n\n\n\n')
        elif buffer == 'save' and self.header is not None:
            self.save_data()
        elif isinstance(buffer, str) and buffer == 'complete':
            print('All data received.')
            df = pd.DataFrame(self.data, columns=self.header)
            print(df)

    def save_data(self):
        # Save the current data to CSV and PDF
        filepath = 'LivePlot' + str(datetime.now().strftime('_%Y-%m-%d_%H-%M-%S'))
        df = pd.DataFrame(self.data, columns=self.header)
        df.to_csv(filepath + '.csv', index=False)
        plt.savefig(filepath + '.pdf')
        print(f'\nLive Plot saved: {filepath}.pdf')
        print(f'Live Data saved: {filepath}.csv\n\n\n\n')

    def update_plots(self):
        # Update the plots with the latest data
        if len(self.data) > 0:
            df = pd.DataFrame(self.data, columns=self.header)
            time = df.index
            colors = plt.rcParams["axes.prop_cycle"]()

            for i in range(len(self.header)):
                data_row = df.iloc[:, i]
                color = next(colors)["color"]

                if i < 5:
                    self.update_timeline(i, time, data_row, color)
                    self.update_histogram(i, data_row, color)

    def update_timeline(self, i, time, data_row, color):
        # Update timeline plot
        self.axs[i, 0].clear()
        self.axs[i, 0].set_title(
            f"Min={data_row.min():.3f}, Max={data_row.max():.3f}, Mean={data_row.mean():.3f}, Std={data_row.std():.5f}, N={data_row.size}",
            size=8)
        self.axs[i, 0].plot(time, data_row, label=self.header[i], marker='.', color=color, markersize=3, linewidth=0.2)
        self.axs[i, 0].legend(loc='upper right')
        self.axs[i, 0].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)

    def update_histogram(self, i, data_row, color):
        # Update histogram plot
        self.axs[i, 1].clear()
        self.axs[i, 1].hist(data_row, bins=50, label=self.header[i], color=color)
        self.axs[i, 1].set_ylabel('Occurrences')
        self.axs[i, 1].set_title(self.header[i])
        self.axs[i, 1].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)

    def set_keep_samples(self, keep_samples):
        self.keep_samples = keep_samples

    def run_standalone(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=200)
        plt.show()
        print('Finished')


if __name__ == '__main__':
    plotter = LivePlotter()
    plotter.run_standalone()
