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
# Each measurement with 1000 angle samples
UpDown = [
[3050.296, 1007.897],
[3047.168, 1009.128],
[3052.019, 1008.827],
[3048.309, 1008.412],
[3051.938, 1007.98],
[3050.3, 1009.017],
[3050.501, 1010.983],
[3048.645, 1009.269],
[3049.029, 1008.778],
[3053.313, 1011.908],
]
UpDown = np.array(UpDown)

average = np.mean(UpDown, axis=0)
std = np.std(UpDown, axis=0)
diff = UpDown[:, 0] - UpDown[:, 1]

full_circle_in_adc_units = 2*diff
full_circle_in_adc_units_avg = np.mean(full_circle_in_adc_units)
full_circle_in_adc_units_std = np.std(full_circle_in_adc_units)

print(f"Full circle in ADC units is {full_circle_in_adc_units_avg} +/- {full_circle_in_adc_units_std} (1 std)")

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
plt.title('Deviation from mean')
plt.plot(UpDown[:, 0]-average[0], label='Equilibrium 1')
plt.plot(UpDown[:, 1]-average[1], label='Equilibrium 2')
plt.plot(full_circle_in_adc_units-full_circle_in_adc_units_avg, label='Full circle')
plt.legend()
plt.show()