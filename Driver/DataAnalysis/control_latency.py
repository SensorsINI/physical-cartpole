import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import scipy
import numpy as np

sns.set()
sns.color_palette()


def reject_outliers(data, m=3.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / mdev if mdev else 0.
    return data[s < m]


if __name__ == "__main__":
    # results = {
    #     'PID Python':   'OLD/CP_PID_2022-02-15_02-14-47 Stabilizing.csv',
    #     'MPPI Euler':   'OLD/CP_mppi-tf-Euler_2022-02-15_02-41-29 Stabilizing.csv',
    #     'MPPI RNN':     'OLD/CP_mppi-tf-RNN_2022-02-15_02-55-52 Stabilizing.csv',
    # }
    results = {
        'PID Python':   'CP_PID-RNN_2022-02-21_00-01-11 Performance 10k.csv',
        'MPPI Euler':   'CP_mppi-tf-Euler_2022-02-20_23-53-15 Performance 10k.csv',
        'MPPI RNN':     'CP_mppi-tf-RNN_2022-02-20_23-57-09 Performance 10k.csv',
    }

    firmware_latency = [1000*pd.read_csv('ExperimentRecordings/Performance Measurement/' + results[name], comment='#')['latency'] for name in results]
    python_latency = [1000*pd.read_csv('ExperimentRecordings/Performance Measurement/' + results[name], comment='#')['pythonLatency'] for name in results]
    controller_steptime = [1000*pd.read_csv('ExperimentRecordings/Performance Measurement/' + results[name], comment='#')['controller_steptime'] for name in results]
    delta_time = [pd.read_csv('ExperimentRecordings/Performance Measurement/' + results[name], comment='#')['deltaTimeMs'] for name in results]
    delta_goal = [6, 21, 21]
    misses = [np.count_nonzero(delta_time[i].to_numpy() > delta_goal[i]) * 100 / len(delta_time[i]) for i, name in enumerate(results)]

    fig1 = plt.figure(1)
    fig2 = plt.figure(2)
    gs = fig2.add_gridspec(2, 1)
    ax2 = fig2.add_subplot(gs[0])
    ax3 = fig2.add_subplot(gs[1])

    axes = [fig1.gca(), ax2, ax3]
    colors = ['g', 'b', 'y']

    limits = [(2.5, 5), (8, 20), (8, 20)]
    time = [firmware_latency[0], controller_steptime[1], controller_steptime[2]]
    m = [10, 4, 4]

    for i, name in enumerate(results):
        print(f'Length: {len(firmware_latency[i])}, firmware latency [μ={firmware_latency[i].mean():.2f}ms, σ={firmware_latency[i].std():.2f}ms], python latency [μ={python_latency[i].mean():.2f}ms, σ={python_latency[i].std():.2f}ms], controller step time [μ={controller_steptime[i].mean():.2f}ms, σ={controller_steptime[i].std():.2f}ms], misses:{misses[i]:.2f}%')

        x = reject_outliers(time[i], m=m[i])
        values, bins = np.histogram(x, bins=40, density=True)
        centers = 0.5 * (bins[1:] + bins[:-1])
        width = (bins[1:] - bins[:-1]).mean()

        axes[i].bar(centers, values, width=width, label=f'{name} $(μ={x.mean():.1f}ms, σ={x.std():.1f}ms)$', color=colors[i])
        axes[i].set_xlim(limits[i])
        axes[i].axvline(x=x.mean(), color='black', linestyle='--', alpha=0.7)
        axes[i].annotate(f'{x.mean():.1f}', (x.mean()*1.01, 0.9 * values.max()))

    axes[0].set_ylabel('Density')
    fig1.text(0.5, 0.02, 'End-to-End Latency in ms', ha='center', va='center')
    fig1.legend(loc='upper left', prop={'size': 9})
    fig1.savefig('ExperimentRecordings/Plots/Latency PID.pdf')
    fig1.show()

    axes[1].set_ylabel('Density')
    fig2.text(0.5, 0.02, 'Calculation Time in ms', ha='center', va='center')
    fig2.legend(loc='upper left', prop={'size': 9})
    fig2.savefig('ExperimentRecordings/Plots/Latency Euler & RNN.pdf')
    fig2.show()



