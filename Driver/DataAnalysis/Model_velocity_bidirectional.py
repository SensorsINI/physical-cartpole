import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from sklearn.linear_model import LinearRegression

# Change backend for matplotlib to plot interactively in pycharm
# This import must go before pyplot
from matplotlib import use
# use('TkAgg')
use('macOSX')

MOTOR_RANGE = 8192.0  # One side, it changes in [-MOTOR_RANGE, MOTOR_RANGE]
PATH_TO_DATA = 'ExperimentRecordings/'

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

def acceleration_no_offset_formula(Q,v, parameters):
    a, b = parameters
    return a*Q-b*v

class Motor_Calibration:
    def __init__(self):
        self.Q_normed = None

    def load_data(self, file_path, Q_normed):
        data: pd.DataFrame = pd.read_csv(file_path, comment='#')

        # Calculate dt
        data['dt'] = data['time'].shift(-1) - data['time']
        data['positionD_last'] = data['positionD'].shift(1)
        data['positionD_smoothed'] = smooth(data['positionD'], 30)
        data = data.iloc[1:-1]
        data = data.reset_index(drop=True)

        if Q_normed:
            data['Q'] = data['Q'] * MOTOR_RANGE

        self.data = data
        self.time = self.data['time'].to_numpy()
        self.Q = self.data['Q'].to_numpy()
        self.dt = self.data['dt'].to_numpy()

    def set_parameters(self, slope, b):


        a = slope * b * MOTOR_RANGE
        self.parameters = (a, b)

    def predict_v(self):

        self.v = np.zeros_like(self.time)
        for i in range(len(self.v) - 1):
            if i == 0:
                self.v[0] = 0.0
            self.v[i + 1] = self.v[i] + acceleration_no_offset_formula(self.Q[i] / MOTOR_RANGE, self.v[i], self.parameters) * self.dt[i]

    def plot_predicted_v(self):

        fontsize_labels = 14
        fontsize_ticks = 12

        fig, axs = plt.subplots(2, 1, figsize=(16, 9), sharex=True)

        n = 100
        self.data = self.data.head(int(len(self.data) * (n / 100)))

        axs[0].set_ylabel("Speed (m/s)", fontsize=fontsize_labels)
        axs[0].plot(self.time, self.data['positionD'],
                    'g', markersize=12, label='Velocity')
        axs[0].plot(self.time[:int(len(self.time) * n / 100.0)], self.v[:int(len(self.time) * n / 100.0)],
                    'orange', markersize=12, label='Velocity modeled')
        axs[0].tick_params(axis='both', which='major', labelsize=fontsize_ticks)
        axs[0].legend()

        axs[1].set_ylabel("Motor Input Q (-)", fontsize=fontsize_labels)
        axs[1].plot(self.time, self.Q,
                    'r', markersize=12, label='Q')
        axs[1].tick_params(axis='both', which='major', labelsize=fontsize_ticks)

        axs[1].set_xlabel('Time (s)', fontsize=fontsize_labels)

        fig.align_ylabels()
        plt.tight_layout()
        plt.show()

    def group_by_Q(self, motor_correction):

        # Cut dataset in two:

        a = self.data.Q.idxmax()
        b = self.data.Q.idxmin()
        cutting_index = min(a,b)+200
        if cutting_index == self.data.Q.idxmin() + 200:
            starting_direction = 'neg'
        else:
            starting_direction = 'pos'

        if starting_direction == 'pos':
            data_pos = self.data[:cutting_index]
            data_neg = self.data[cutting_index:]
        else:
            data_pos = self.data[cutting_index:]
            data_neg = self.data[:cutting_index]

        gb_pos = data_pos.groupby(['Q'], as_index=False)
        data_stat_pos = gb_pos.size().reset_index()

        gb_neg = data_neg.groupby(['Q'], as_index=False)
        data_stat_neg = gb_neg.size().reset_index()


        data_stat_pos['v_max'] = gb_pos['positionD_smoothed'].max()['positionD_smoothed']
        data_stat_pos = data_stat_pos[data_stat_pos['Q'] > 0]
        data_stat_linear_pos = data_stat_pos[data_stat_pos['Q'] < MOTOR_RANGE / 2.2]

        data_stat_neg['v_max'] = gb_neg['positionD_smoothed'].min()['positionD_smoothed']
        data_stat_neg = data_stat_neg[data_stat_neg['Q'] < 0]
        data_stat_linear_neg = data_stat_neg[data_stat_neg['Q'] > -MOTOR_RANGE / 2]

        Q_neg, v_neg, Q_pred_neg, v_max_pred_neg = self.fit_line(data_stat_linear_neg, data_stat_neg)
        Q_pos, v_pos, Q_pred_pos, v_max_pred_pos = self.fit_line(data_stat_linear_pos, data_stat_pos)

        self.Q_sparse, self.v_sparse, self.Q_pred_sparse, self.v_max_pre_sparse = \
            np.concatenate((Q_neg-motor_correction, Q_pos+motor_correction)), np.concatenate((v_neg, v_pos)), \
            np.concatenate((Q_pred_neg, Q_pred_pos)), np.concatenate((v_max_pred_neg, v_max_pred_pos)),


        return self.Q_sparse, self.v_sparse, self.Q_pred_sparse, self.v_max_pre_sparse

    def fit_line(self, data_stat_linear, data_stat):

        motor_for_fit = data_stat_linear['Q'].to_numpy().reshape(-1, 1)
        v_max_for_fit = data_stat_linear['v_max'].to_numpy().reshape(-1, 1)
        reg = LinearRegression().fit(motor_for_fit, v_max_for_fit)

        slope = reg.coef_[0][0]
        intercept_y = reg.intercept_[0]
        intercept_x = -intercept_y / slope
        print('Coefficients of linear regression:')
        print('Slope:')
        print(np.around(slope, 7))
        print('Intercept Y:')
        print(np.around(intercept_y, 2))
        print('Intercept X:')
        print(np.around(intercept_x, 2))

        motor_for_prediction = data_stat['Q'].to_numpy().reshape(-1, 1)
        motor_for_prediction = np.concatenate(
            (-500 * np.ones(shape=(1, 1)), np.zeros(shape=(1, 1)), 500 * np.ones(shape=(1, 1)), motor_for_prediction))

        v_max_predicted = reg.predict(motor_for_prediction)
        Q = data_stat['Q'].to_numpy()[:, np.newaxis]
        v = data_stat['v_max'].to_numpy()[:, np.newaxis]
        Q_pred = motor_for_prediction

        return Q, v, Q_pred, v_max_predicted

    def plot_v_sat(self, slope=0.000127, title=None):
        Q, v, Q_pred, v_pred = self.Q_sparse, self.v_sparse, self.Q_pred_sparse, self.v_max_pre_sparse

        Q_ideal = np.array([-max(abs(Q)), max(abs(Q))])
        v_ideal = slope * Q_ideal

        plt.figure(figsize=(16, 9))
        plt.scatter(Q, v, marker='o', label='Measured v_saturation')
        plt.plot(Q_ideal, v_ideal, marker='', label='Linear regression for small v_saturation')
        plt.xlabel('Motor input Q [-]')
        plt.ylabel('Maximal reached speed [m/s]')
        if title is None:
            title = 'Q vs v_sat'
        else:
            title = 'Q vs v_sat: '+title
        plt.title(title)
        plt.legend()
        plt.grid()
        plt.show()

    def process_file(self, file_path, motor_correction=0.0, Q_normed=True, slope=0.000127, b=20):
        self.load_data(file_path, Q_normed)
        self.set_parameters(slope, b)
        self.predict_v()
        # self.plot_predicted_v()
        Q, v, Q_pred, v_max_predicted = self.group_by_Q(motor_correction=motor_correction)
        return Q, v, Q_pred, v_max_predicted



if __name__ == '__main__':

    # file_path = PATH_TO_DATA + 'left-CP-corrected.csv'
    # file_path = PATH_TO_DATA + 'right-CP-corrected.csv'
    # file_path = PATH_TO_DATA + 'left-CP-not-corrected.csv'
    # file_path = PATH_TO_DATA + 'right-CP-not-corrected.csv'

    # if 'right' in file_path or 'pololu' in file_path:
    #     title = 'Pololu'
    #     slope = 0.000108  # Polulu
    # elif 'left' in file_path or 'original' in file_path:
    #     title = 'Original'
    #     slope = 0.000127  # Normal
    # else:
    #     title = None
    #
    #
    #
    # if 'not-corrected' in file_path:
    #     motor_correction = -0.0
    # elif 'Pololu' in title:
    #     motor_correction = -200.0
    # elif 'Original' in title:
    #     motor_correction = -375.0

    # correction_for_motor = -375.0
    # intermediate - overwrite the above
    # correction_for_motor = -300.0

    # MCI = Motor_Calibration()

    # Q, v, Q_pred, v_pred = MCI.process_file(file_path, Q_normed=True, motor_correction=motor_correction)

    # MCI.plot_v_sat(slope=slope, title=title)



    # Plot for meeting - comparison of the two motors
    file_path = PATH_TO_DATA + 'left-CP-not-corrected.csv'
    slope_n = 0.000127  # Normal
    correction_for_motor_n = -375.0
    # correction_for_motor_n = -0.0
    MCI = Motor_Calibration()
    Q_original, v_original, Q_pred_original, v_pred_original = MCI.process_file(file_path, Q_normed=True, motor_correction=correction_for_motor_n)
    Q_ideal_n = np.array([-max(abs(Q_original)), max(abs(Q_original))])
    v_ideal_n = slope_n * Q_ideal_n

    file_path = PATH_TO_DATA + 'right-CP-not-corrected.csv'
    slope_p = 0.000108  # Polulu
    correction_for_motor_p = -200
    # correction_for_moto_p = -0.0
    MCI = Motor_Calibration()
    Q_pololu, v_pololu, Q_pred_pololu, v_pred_pololu = MCI.process_file(file_path, Q_normed=True, motor_correction=correction_for_motor_p)
    Q_pololu_p = np.array([-max(abs(Q_pololu)), max(abs(Q_pololu))])
    v_ideal_p = slope_p * Q_pololu_p

    plt.figure(figsize=(16, 9))

    original = plt.scatter(Q_original, v_original, marker='o', label='v_saturation - original motor', color='b')
    plt.plot(Q_ideal_n, v_ideal_n, marker='', label='Linear regression for small v_saturation', color='lightblue')

    pololu = plt.scatter(Q_pololu[2:-2], v_pololu[2:-2], marker='o', label='v_saturation - pololu motor', color='green')
    plt.plot(Q_pololu_p, v_ideal_p, marker='', label='Linear regression for small v_saturation', color='lightgreen')

    plots = [pololu, original]
    labs = [pololu.get_label(), original.get_label()]

    plt.xlabel('Motor input Q [-]')
    plt.ylabel('Maximal reached speed [m/s]')
    plt.title('$Q$ vs. $v_{sat}$')
    plt.legend(plots, labs)
    plt.grid()
    plt.show()