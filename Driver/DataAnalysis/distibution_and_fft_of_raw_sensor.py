from tqdm import tqdm
import matplotlib.pyplot as plt
import scipy
from scipy.signal.windows import blackman
import numpy as np
import seaborn as sns

from DriverFunctions.interface import Interface
from DriverFunctions.interface import get_serial_port
from globals import CHIP, SERIAL_PORT_NUMBER, SERIAL_BAUD

sns.set()
sns.color_palette()


def plot_distribution(x, caption='Histogramm'):
    mean = int(x.mean())
    # Bins
    bins = 60
    min_bin = mean - bins / 2
    max_bin = mean + bins / 2
    bins = np.arange(min_bin, max_bin + 1, 1)

    # Gaussian Distribution
    x_gauss = bins - 0.5
    y_gauss = scipy.stats.norm(loc=mean, scale=x.std()).pdf(x_gauss)

    # Plotting
    # plt.title(f'Histogram of {len(x):,} Samples')
    plt.plot(x_gauss, y_gauss, color='orange', label=f'$\mathcal{{N}}(μ={x.mean():.1f}, σ={x.std():.1f})$', linestyle='--', alpha=0.8)
    plt.hist(x, bins=bins, range=(min_bin, max_bin), label='Sensor Data', density=True)
    plt.xlabel('ADC Output (12-Bit)')
    plt.ylabel('Density')
    plt.xlim([min_bin, max_bin])
    plt.legend()
    plt.savefig('ExperimentRecordings/Plots/' + caption + '.pdf')
    plt.show()


def plot_fft(series, timestep, caption='FFT'):
    n = len(series[0])
    coefficients = np.zeros((len(series), n // 2))

    for i, x in enumerate(series):
        signal = x - x.mean()
        coefficients[i] = 2.0 / n * np.abs(scipy.fft(signal * blackman(n))[:n // 2])

    coefficients = coefficients.mean(axis=0)

    gaussian = scipy.ndimage.filters.gaussian_filter(coefficients, sigma=10, mode="constant", cval=0)
    savgol = scipy.signal.savgol_filter(coefficients, window_length=501, polyorder=5)
    freq = np.fft.fftfreq(n, d=timestep)[:n // 2]

    # plt.title(f'Discrete Fourier Transform of {len(x):,} Samples')
    plt.semilogy(freq, coefficients, linewidth=1, marker='.', markersize=2, label='Blackman FFT')
    #plt.semilogy(freq, gaussian, linewidth=1, label='Gaussian Blur')
    # plt.semilogy(freq, savgol, linewidth=1, label='Savitzky-Golay Filter')
    plt.legend()
    plt.xlabel('Frequency in Hz')
    plt.ylabel('Discrete Fourier Coefficients')
    plt.ylim([0.01, 1])
    plt.savefig('ExperimentRecordings/Plots/' + caption + '.pdf')
    plt.show()


if __name__ == "__main__":
    InterfaceInstance = Interface()
    SERIAL_PORT = get_serial_port(chip_type=CHIP, serial_port_number=SERIAL_PORT_NUMBER)
    InterfaceInstance.open(SERIAL_PORT, SERIAL_BAUD)
    InterfaceInstance.control_mode(False)
    InterfaceInstance.stream_output(False)

    timeseries = []
    for i in tqdm(range(20)):
        timeseries.append(np.array(InterfaceInstance.collect_raw_angle(lenght=8000, interval_us=100)))

    plot_distribution(np.concatenate(timeseries), caption='ADC Histogramm')
    plot_fft(timeseries, timestep=200e-6, caption='ADC FFT')
