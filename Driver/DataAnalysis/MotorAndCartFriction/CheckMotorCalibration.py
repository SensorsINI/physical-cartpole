"""
This script, in big part similar to MotorCalibration.py,
let you check if the calibration done with MotorCalibration.py brought intended results.
It plots v_sat vs. Q together with line corresponding to v_sat = v_sat_max * Q
(where v_sat_max must be provided by user)
and
calculates what would be v_sat_max if it were to be determined from current data.
"""
import matplotlib.pyplot as plt

from double_regression import double_regression
import numpy as np
import pandas as pd
import platform

# Change backend for matplotlib to plot interactively in pycharm
from matplotlib import use

from sklearn.linear_model import LinearRegression

if platform.system() == 'Darwin':
    use('macOSX')
else:
    use('TkAgg')

DATA_SMOOTHING = 2  # May strongly influence what is the max velocity and hence the results of calibration

# Define the variables
FILE_NAME = 'Original.csv'
# FILE_NAME = 'Pololu.csv'

PATH_TO_DATA = 'DataAnalysis/MotorAndCartFriction/'
file_path = PATH_TO_DATA + FILE_NAME

v_max_sat = 0.55

# Define functions

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

# Load data

data: pd.DataFrame = pd.read_csv(file_path, comment='#')

data['dt'] = data['time'].shift(-1) - data['time']
data['positionD_last'] = data['positionD'].shift(1)
data['positionD_smoothed'] = smooth(data['positionD'], DATA_SMOOTHING)

data = data.iloc[1:-1]
data = data.reset_index(drop=True)


# Cut data into part when cart accelerates in positive and negative direction

Q_idx_min = data.Q.idxmin()
Q_idx_max = data.Q.idxmax()

Q_max = data.Q.max()
Q_min = data.Q.min()

cutting_index = min(Q_idx_min, Q_idx_max) + 400
if cutting_index == Q_idx_min + 400:
    starting_direction = 'neg'
else:
    starting_direction = 'pos'

if starting_direction == 'pos':
    data_pos = data[:cutting_index]
    data_neg = data[cutting_index:]
else:
    data_pos = data[cutting_index:]
    data_neg = data[:cutting_index]

gb_pos = data_pos.groupby(['Q'], as_index=False)
data_stat_pos = gb_pos.size().reset_index(drop=True)
data_stat_pos['v_max'] = gb_pos['positionD_smoothed'].max()['positionD_smoothed']
data_stat_pos = data_stat_pos[data_stat_pos['Q'] > 0]

gb_neg = data_neg.groupby(['Q'], as_index=False)
data_stat_neg = gb_neg.size().reset_index(drop=True)
data_stat_neg['v_max'] = gb_neg['positionD_smoothed'].min()['positionD_smoothed']
data_stat_neg = data_stat_neg[data_stat_neg['Q'] < 0]

data_stat = pd.concat([data_stat_pos, data_stat_neg], ignore_index=True)

Q = data_stat['Q'].to_numpy()
v_sat = data_stat['v_max'].to_numpy()

data_stat_lin = data_stat[data_stat['Q'].abs() < 1.0]

Q_lin = data_stat_lin['Q'].to_numpy()
v_sat_lin = data_stat_lin['v_max'].to_numpy()

# Linear regression
reg = LinearRegression().fit(np.atleast_2d(Q_lin).T, np.atleast_2d(v_sat_lin).T)

v_sat_max_current = reg.coef_[0][0]

print('Regression parameters:')
print('v_sat = v_sat_max_current * Q')
print('v_sat_max_current = {}'.format(v_sat_max_current))

# plot
plt.figure()
plt.title(FILE_NAME)

Q_axis = np.linspace(-1.1, 1.1, 100)
v_sat_axis = v_max_sat * Q_axis

plt.plot(Q_axis, v_sat_axis)
plt.scatter(Q, v_sat)
plt.scatter(Q_lin, v_sat_lin)
plt.show()

