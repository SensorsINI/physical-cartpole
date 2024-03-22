import numpy as np
import matplotlib.pyplot as plt


UpDown = [  # STM
[3200.988, 1062.653],
[3200.455, 1063.622],
[3198.458, 1063.781],
[3197.906, 1061.552],
[3198.045, 1063.804],
[3196.103, 1063.105],
[3196.197, 1061.163],
[3197.592, 1062.099],
[3197.778, 1060.736],
[3197.314, 1062.049],
[3196.404, 1060.326],
]

# Zybo
UpDown = [
[2989.559,	940.132],
[2991.048,	942.507],
[2992.806,	941.31],
]
UpDown = np.array(UpDown)

average = np.mean(UpDown, axis=0)
std = np.std(UpDown, axis=0)
diff = UpDown[:, 0] - UpDown[:, 1]

full_circle_in_adc_units = 2*diff
full_circle_in_adc_units_avg = np.mean(full_circle_in_adc_units)
full_circle_in_adc_units_std = np.std(full_circle_in_adc_units)

print(f"Full circle in ADC units is {full_circle_in_adc_units_avg} +/- {full_circle_in_adc_units_std}")

adc_range = 4096
adc_units_2_deg = 360.0/adc_range
adc_units_2_rad = 2*np.pi/adc_range

angle_std_rad = (adc_units_2_rad**2)*full_circle_in_adc_units_std
angle_std_deg = (adc_units_2_deg**2)*full_circle_in_adc_units_std

# For dead angle only average
dead_angle_in_adc_units = full_circle_in_adc_units_avg-adc_range
dead_angle_rad = dead_angle_in_adc_units*adc_units_2_rad
dead_angle_deg = dead_angle_in_adc_units*adc_units_2_deg

print(f"Dead angle is at least {dead_angle_deg} deg")

pass


plt.figure()
plt.plot(UpDown[:, 0]-average[0], label='Up')
plt.plot(UpDown[:, 1]-average[1], label='Down')
plt.plot(full_circle_in_adc_units-full_circle_in_adc_units_avg, label='Diff')
plt.legend()
plt.show()