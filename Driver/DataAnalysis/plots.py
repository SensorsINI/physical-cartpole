import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy
import glob
import os
import seaborn as sns
sns.set()
from datetime import datetime
from scipy import asarray as ar, exp, sqrt
from scipy.optimize import curve_fit


def plot_distribution(x, caption='Histogramm', xlabel=''):
    # Bins
    min_bin = np.min(x)
    max_bin = np.max(x)
    bins = np.arange(min_bin, max_bin + 1, 1)
    #values, bins = np.histogram(x, bins=100)

    # Gaussian Distribution
    #x_gauss = np.arange(min_bin - 10, max_bin + 10, 1)
    x_gauss = bins
    y_gauss = len(x) * scipy.stats.norm(loc=x.mean(), scale=x.std()).pdf(bins)

    # Plotting
    plt.hist(x, bins=len(bins))
    plt.plot(x_gauss, y_gauss, color='orange')
    plt.xlabel(xlabel)
    plt.savefig('ExperimentRecordings/Plots/'+caption+datetime.now().strftime(" %Y-%m-%d-%H-%M-%S.pdf"))
    plt.show()


def plot_fft(x, timestep, caption='FFT'):
    signal = x - x.mean()
    N = len(signal)

    from scipy.signal import blackman
    X_windowed = 2.0/N * np.abs(scipy.fft(signal * blackman(N))[:N//2])
    gaussian = scipy.ndimage.filters.gaussian_filter(X_windowed, sigma=10, mode="constant", cval=0)
    savgol = scipy.signal.savgol_filter(X_windowed, window_length=501, polyorder=5)
    freq = np.fft.fftfreq(N, d=timestep)[:N//2]

    plt.semilogy(freq, X_windowed, linewidth=1, marker='.', markersize=2)
    plt.semilogy(freq, gaussian, linewidth=1)
    plt.semilogy(freq, savgol, linewidth=1)
    plt.legend(['Blackman FFT', 'Gaussian Blur', 'Savitzky-Golay Filter'])
    plt.xlabel('Frequency [Hz]')
    plt.savefig('ExperimentRecordings/Plots/'+caption+datetime.now().strftime(" %Y-%m-%d-%H-%M-%S.pdf"))
    plt.show()


if __name__ == "__main__":
    list_of_files = glob.glob('ExperimentRecordings/*.csv')
    latest_file = max(list_of_files, key=os.path.getctime)

    data = pd.read_csv(latest_file, comment='#')  # skip comment lines starting with #
    angle_raw_data = data['angle_raw']
    plot_fft(angle_raw_data.numpy())
