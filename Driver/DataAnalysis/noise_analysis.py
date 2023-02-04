import csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import glob
import os

list_of_files = glob.glob('../ExperimentRecordings/*.csv')
latest_file = max(list_of_files, key=os.path.getctime)

data: pd.DataFrame = pd.read_csv(latest_file, comment='#')  # skip comment lines starting with #
angle_raw_data = data['angle_raw']

# extract data information
num_data = len(angle_raw_data)
min_bin = np.min(angle_raw_data)
max_bin = np.max(angle_raw_data)
range_data = max_bin
var = angle_raw_data.var()
mean = angle_raw_data.mean()
std = np.sqrt(var)

# histogram plotting
bin_values = np.arange(min_bin, max_bin+1, 1)
bin_num = len(bin_values)

# gaussian distribution
x_gauss = np.arange(min_bin-10, max_bin+10, 1)
y_gauss = stats.norm(mean, std)
amp = 11000 # amplitude of gaussian -  manually find

# plotting
# plt.scatter(angle_raw_data, np.arange(0, len(angle_raw_data), 1))
plt.hist(angle_raw_data, bins = bin_values)
plt.title('Histogram of raw angle data, bin_num = %i' % bin_num)
plt.plot(x_gauss, amp*y_gauss.pdf(x_gauss))
plt.show()