import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

list_of_files = glob.glob('ExperimentRecordings/*.csv')
latest_file = max(list_of_files, key=os.path.getctime)
data = pd.read_csv(latest_file, comment='#')

data = data[['angle', 'angle_raw']][0:2000]

fig, axs  = plt.subplots(len(data.columns), 1, figsize=(12, 9))
fig2, axs2 = plt.subplots(len(data.columns), 1, figsize=(12, 9))
colors = plt.rcParams["axes.prop_cycle"]()

print("\n")
print(f"Analysing File: {latest_file}")
print(f"Angle Raw -- Min: {data['angle_raw'].min()}, Max: {data['angle_raw'].max()}, Mean: {data['angle_raw'].mean():.3f}, Std: {data['angle_raw'].std():.3f}")
print()

for i, col_name in enumerate(data):
    color = next(colors)["color"]

    axs[i].hist(data[col_name], bins=50, label=col_name, color=color)
    axs[i].set_ylabel('occurences')
    axs[i].set_xlabel(col_name)
    axs[i].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)

    axs2[i].plot(data[col_name], label=col_name, marker='.', color=color)
    axs2[i].legend(loc='upper right')
    axs2[i].grid(True, which='both', linestyle='-.', color='grey', linewidth=0.5)
    
plt. show()