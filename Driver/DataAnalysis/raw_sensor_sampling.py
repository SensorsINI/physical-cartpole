from globals import *
from DriverFunctions.interface import Interface
from plots import plot_distribution, plot_fft

InterfaceInstance = Interface()
InterfaceInstance.open(SERIAL_PORT, SERIAL_BAUD)
InterfaceInstance.control_mode(False)
InterfaceInstance.stream_output(False)

timeseries = np.array(InterfaceInstance.collect_raw_angle(lenght=8000, interval_us=100))

plot_distribution(timeseries, caption='Histogram of Angle (8\'000 Samples, 10kHz)', xlabel='Raw Output of 10 Bit-ADC')
plot_fft(timeseries, timestep=100e-6, caption='FFT of Angle Fluctuation (8\'000 Samples, 10kHz)')
