from globals import *
from DriverFunctions.interface import Interface
from plots import plot_distribution, plot_fft
import pandas as pd
import matplotlib.pyplot as plt

def bmatrix(a):
    """Returns a LaTeX bmatrix

    :a: numpy array
    :returns: LaTeX bmatrix as a string
    """
    if len(a.shape) > 2:
        raise ValueError('bmatrix can at most display two dimensions')
    lines = str(a.to_string(index=False, header=False)).replace('[', '').replace(']', '').splitlines()
    rv = [r'\begin{bmatrix}']
    rv += ['    ' + ' & '.join(l.split()) + r'\\' for l in lines]
    rv += [r'\end{bmatrix}']
    return '\n'.join(rv)

if __name__ == "__main__":
    files = {
        'Median (Length 1, 10kHz)': 'CP_mppi-tf_2022-02-13_00-17-00 Median (Length 1, 10kHz).csv',
        'Median (Length 4, 10kHz)': 'CP_mppi-tf_2022-02-13_00-41-01 Median (Length 4, 10kHz).csv',
        'Median (Length 8, 10kHz)': 'CP_mppi-tf_2022-02-13_00-37-42 Median (Length 8, 10kHz).csv',
        'Median (Length 16, 10kHz)': 'CP_mppi-tf_2022-02-13_00-22-53 Median (Length 16, 10kHz).csv',
        'Median (Length 32, 10kHz)': 'CP_mppi-tf_2022-02-13_00-12-50 Median (Length 32, 10kHz).csv',
        'Median (Length 64, 10kHz)': 'CP_mppi-tf_2022-02-13_00-32-11 Median (Length 64, 10kHz).csv',
        'Median (Length 32, 10kHz, 5ms)': 'CP_mppi-tf_2022-02-14_18-41-32 Median (Length 32, 10kHz, 5ms.csv',
        #'Median (Length 64_2, 10kHz)': 'CP_mppi-tf_2022-02-14_17-29-47 Median (Lengh 64, 10kHz).csv',
        #'Median (Length 128, 10kHz)': 'CP_mppi-tf_2022-02-14_17-23-28 Median (Length 128, 10kHz).csv',
    }

    #np.set_printoptions(suppress=False)
    pd.options.display.float_format = '{0:.6f}^2'.format

    trajectories = [pd.read_csv('ExperimentRecordings/Noise Measurements/' + files[name], comment='#') for name in files]

    for i, name in enumerate(files):
        trajectory = trajectories[i]
        plt.plot(trajectory['angle_raw'], label=name)

        print(name)
        rows = ['angle', 'angleD', 'position', 'positionD']
        variances = [trajectory[row].std() for row in rows]
        cov = trajectory[rows].cov().round(decimals=6)
        print(bmatrix(np.sqrt(cov)))
        print(variances)

    plt.legend(prop={'size': 8})
    plt.savefig('ExperimentRecordings/Plots/Median Filter.pdf')
    plt.show()

    trajectories = [pd.read_csv('ExperimentRecordings/Noise Measurements/' + files[name], comment='#') for name in files]
