import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime
import pandas as pd
import seaborn as sns

from live_plotter_connection_handler import ConnectionHandler

sns.set()

DEFAULT_FEATURES_TO_PLOT = 'default'  # None, 'default', list of features

KEEP_SAMPLES_DEFAULT = 100  # at least 10
DEFAULT_ADDRESS = ('0.0.0.0', 6000)

class LivePlotter:
    def __init__(self, address=DEFAULT_ADDRESS, keep_samples=KEEP_SAMPLES_DEFAULT, header_callback=None):
        # Set up connection handler for incoming data
        self.connection_handler = ConnectionHandler(address)
        self.data = []
        self.header = None
        self.received = 0
        self.keep_samples = keep_samples
        self.selected_features = ['None'] * 5  # Default to 'None' for all subplots
        self.header_callback = header_callback  # Callback function for headers

        self.fig, axs = plt.subplots(5, 2, figsize=(16, 9), gridspec_kw={'width_ratios': [3, 1]})

        self.fig.subplots_adjust(hspace=0.8)
        self.fig.canvas.manager.set_window_title('Live Plot')

        if not isinstance(axs, np.ndarray):
            axs = np.atleast_1d(axs)
        self.axs = axs

        self.animation = None

    def animate(self, i):
        for buffer in self.connection_handler.poll_connection():
            self.process_buffer(buffer)

        if self.received >= 10:
            self.received = 0
            self.update_plots()

    def process_buffer(self, buffer):
        # Process incoming buffer based on its type
        if isinstance(buffer, list) and isinstance(buffer[0], str):
            self.header = buffer
            print(f'Header received: {self.header}')
            self.reset_liveplotter()
            if DEFAULT_FEATURES_TO_PLOT == 'default':
                self.selected_features = self.header[:5] + ['None'] * (5 - len(self.header))  # Default first 5 headers
            elif DEFAULT_FEATURES_TO_PLOT is not None:
                self.selected_features = [feature for feature in DEFAULT_FEATURES_TO_PLOT if feature in self.header]
                self.selected_features = self.selected_features + ['None'] * (5 - len(self.selected_features))
            else:
                self.selected_features = ['None'] * 5

            # Update the subplot layout based on selected features
            self.update_subplot_layout()

            if self.header_callback:
                self.header_callback(self.header, self.selected_features)  # Call the callback with the new headers
        elif isinstance(buffer, np.ndarray):
            self.data.append(buffer)
            self.received += 1
            self.data = self.data[-self.keep_samples:]
        elif buffer == 'reset':
            self.reset_liveplotter()
        elif buffer == 'save' and self.header is not None:
            self.save_data()
        elif isinstance(buffer, str) and buffer == 'complete':
            print('All data received.')

    def reset_liveplotter(self):
        self.data = []
        print('\nLive Plot Reset\n\n\n\n')

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

            for i, feature in enumerate(self.selected_features):
                if feature in self.header:
                    data_row = df[feature]
                    color = next(colors)["color"]
                    self.update_timeline(i, time, data_row, color)
                    self.update_histogram(i, data_row, color)

    def clear_subplot(self, i):
        self.axs[i, 0].clear()
        self.axs[i, 1].clear()

    def update_timeline(self, i, time, data_row, color):
        # Update timeline plot
        self.axs[i, 0].clear()
        self.axs[i, 0].set_title(
            f"Min={data_row.min():.3f}, Max={data_row.max():.3f}, Mean={data_row.mean():.3f}, Std={data_row.std():.5f}, N={data_row.size}",
            size=8)
        self.axs[i, 0].plot(time, data_row, label=self.selected_features[i], marker='.', color=color, markersize=3, linewidth=0.2)
        self.axs[i, 0].legend(loc='upper right')
        self.axs[i, 0].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)

    def update_histogram(self, i, data_row, color):
        # Update histogram plot
        self.axs[i, 1].clear()
        self.axs[i, 1].hist(data_row, bins=50, label=self.selected_features[i], color=color)
        self.axs[i, 1].set_ylabel('Occurrences')
        self.axs[i, 1].set_title(self.selected_features[i])
        self.axs[i, 1].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)

    def set_keep_samples(self, keep_samples):
        self.keep_samples = keep_samples

    def update_selected_features(self, features):
        self.selected_features = features
        self.update_subplot_layout()  # Update the subplot layout based on new selected features

    def update_subplot_layout(self):
        self.fig.clf()  # Clear the current figure
        subplots_count = max(1, len([feature for feature in self.selected_features if (feature is not None and feature!='None')]))
        axs = self.fig.subplots(subplots_count, 2, gridspec_kw={'width_ratios': [3, 1]})  # Create new subplots
        if not isinstance(axs, np.ndarray):
            axs = np.atleast_1d(axs)
        self.axs = axs

    def run_standalone(self):
        self.animation = animation.FuncAnimation(self.fig, self.animate, interval=200)
        plt.show()
        print('Finished')


if __name__ == '__main__':
    plotter = LivePlotter()
    plotter.run_standalone()
