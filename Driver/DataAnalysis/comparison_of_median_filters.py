from globals import *
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

sns.set()
sns.color_palette()


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
    advanced10k = {
        'Advanced Median (Length 1, 10kHz, Pole Up, 20ms)': 'CP_mppi-tf_2022-02-13_00-17-00 Advanced Median (Length 1, 10kHz, Pole Up, 20ms).csv',
        'Advanced Median (Length 4, 10kHz, Pole Up, 20ms)': 'CP_mppi-tf_2022-02-13_00-41-01 Advanced Median (Length 4, 10kHz, Pole Up, 20ms).csv',
        'Advanced Median (Length 8, 10kHz, Pole Up, 20ms)': 'CP_mppi-tf_2022-02-13_00-37-42 Advanced Median (Length 8, 10kHz, Pole Up, 20ms).csv',
        'Advanced Median (Length 16, 10kHz, Pole Up, 20ms)': 'CP_mppi-tf_2022-02-13_00-22-53 Advanced Median (Length 16, 10kHz, Pole Up, 20ms).csv',
        'Advanced Median (Length 32, 10kHz, Pole Up, 20ms)': 'CP_mppi-tf_2022-02-13_00-12-50 Advanced Median (Length 32, 10kHz, Pole Up, 20ms).csv',
        'Advanced Median (Length 64, 10kHz, Pole Up, 20ms)': 'CP_mppi-tf_2022-02-13_00-32-11 Advanced Median (Length 64, 10kHz, Pole Up, 20ms).csv',
    }
    classic2_5k = {
        'Classic Median (Length 1, 2.5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_22-07-04 Classic Median (Length 1, 2.5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 4, 2.5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_22-04-11 Classic Median (Length 4, 2.5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 8, 2.5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_22-01-00 Classic Median (Length 8, 2.5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 16, 2.5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_21-51-54 Classic Median (Length 16, 2.5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 32, 2.5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_22-11-13 Classic Median (Length 32, 2.5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 64, 2.5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_21-58-02 Classic Median (Length 64, 2.5kHz, Pole Up, 20ms).csv',
    }
    classic5k = {
        'Classic Median (Length 1, 5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_20-49-18 Classic Median (Length 1, 5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 4, 5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_20-46-03 Classic Median (Length 4, 5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 8, 5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_20-42-46 Classic Median (Length 8, 5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 16, 5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_20-39-22 Classic Median (Length 16, 5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 32, 5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_23-21-21 Classic Median 100k (Length 32, 5kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 64, 5kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_20-36-11 Classic Median (Length 64, 5kHz, Pole Up, 20ms).csv',
    }
    classic10k = {
        'Classic Median (Length 1, 10kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_21-09-59 Classic Median (Length 1, 10kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 4, 10kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_21-13-07 Classic Median (Length 4, 10kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 8, 10kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_21-16-08 Classic Median (Length 8, 10kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 16, 10kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_21-19-15 Classic Median (Length 16, 10kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 32, 10kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_21-22-16 Classic Median (Length 32, 10kHz, Pole Up, 20ms).csv',
        'Classic Median (Length 64, 10kHz, Pole Up, 20ms)': 'CP_PID_2022-02-14_21-25-16 Classic Median (Length 64, 10kHz, Pole Up, 20ms).csv',
    }

    pd.options.display.float_format = '{0:.6f}^2'.format

    rows = ['angle', 'angleD', 'position', 'positionD']
    length = [1, 4, 8, 16, 32, 64]

    # Plot Versions vs Length
    std_advanced10k = [pd.read_csv('ExperimentRecordings/Median Filter Measurements/' + advanced10k[name], comment='#')['angle_raw'].std() for name in advanced10k]
    std_classic2_5k = [pd.read_csv('ExperimentRecordings/Median Filter Measurements/' + classic2_5k[name], comment='#')['angle_raw'].std() for name in classic2_5k]
    std_classic5k = [pd.read_csv('ExperimentRecordings/Median Filter Measurements/' + classic5k[name], comment='#')['angle_raw'].std() for name in classic5k]
    std_classic10k = [pd.read_csv('ExperimentRecordings/Median Filter Measurements/' + classic10k[name], comment='#')['angle_raw'].std() for name in classic10k]
    #plt.plot(length, std_advanced10k, label='Advanced Median Filter, Oversampling 10kHz', marker='.', markersize=8)
    plt.plot(length, std_classic2_5k, label='Classic Median Filter, Oversampling 2.5kHz', marker='.', markersize=8)
    plt.plot(length, std_classic5k, label='Classic Median Filter, Oversampling 5kHz', marker='.', markersize=8)
    plt.plot(length, std_classic10k, label='Classic Median Filter, Oversampling 10kHz', marker='.', markersize=8)
    plt.xlabel('Filter Length')
    plt.ylabel('Average Angle Deviation')
    plt.legend(fontsize='medium')
    plt.savefig('ExperimentRecordings/Plots/Median Filter Comparison.pdf')
    plt.show()

    # Covariance for used Filter
    classic32 = pd.read_csv('ExperimentRecordings/Median Filter Measurements/CP_PID_2022-02-14_23-21-21 Classic Median 100k (Length 32, 5kHz, Pole Up, 20ms).csv', comment='#')[:100000]
    print(bmatrix(np.sqrt(classic32[rows].cov())))
    print('')
    print(np.sqrt(classic32[['angle_raw', 'angleD_raw', 'angleD']].cov()))
    print('')
    print(classic32[['angle_raw', 'angleD_raw', 'angleD']].to_numpy().std(axis=0))
