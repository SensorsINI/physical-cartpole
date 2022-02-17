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
    results = {
        'PID Python': 'CP_PID_2022-02-15_02-14-47 Stabilizing.csv',
        'MPPI Euler': 'CP_mppi-tf_2022-02-15_02-41-29 Euler-MPPI Stabilizing.csv',
        'MPPI RNN': 'CP_mppi-tf_2022-02-15_02-55-52 RNN-MPPI Stabilizing.csv',
    }

    latencies = [pd.read_csv('ExperimentRecordings/Results/' + results[name], comment='#')['latency'] for name in results]

    fig = plt.figure()
    gs = fig.add_gridspec(2, 2)

    ax1 = fig.add_subplot(gs[:, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 1])

    ax1.set_xlim(2.5, 3.5)
    ax2.set_xlim(12, 18)
    ax3.set_xlim(12, 18)

    axes = [ax1, ax2, ax3]
    colors = ['g', 'b', 'y']

    for i, name in enumerate(results):
        latency = latencies[i]
        print(len(latency))

        x = reject_outliers(1000*latency)
        values, bins = np.histogram(x, bins=40, density=True)
        centers = 0.5 * (bins[1:] + bins[:-1])
        width = (bins[1:] - bins[:-1]).mean()
        axes[i].bar(centers, values, width=width, label=f'{name} $(μ={x.mean():.1f}ms, σ={x.std():.1f}ms)$', color=colors[i])

        # Fit Gaussian Distribution
        #x_gauss = bins
        #y_gauss = scipy.stats.norm(loc=x.mean(), scale=x.std()).pdf(x_gauss)
        #axes[i].plot(x_gauss, y_gauss, color='orange', linestyle='--', alpha=0.8)

    #plt.tight_layout()
    ax1.set_ylabel('Density')
    fig.text(0.5, 0.02, 'Latency in ms', ha='center', va='center')
    fig.legend(loc='upper left', prop={'size': 9})
    fig.savefig('ExperimentRecordings/Plots/Latencies.pdf')
    fig.show()

