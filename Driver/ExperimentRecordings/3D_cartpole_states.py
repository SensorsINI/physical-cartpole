import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from IROS_Exp1 import get_data, break_line_on_jump

# Load the datasets
dataset_nni_hls = './nc_zynq_v1/iros24-ex1-experiment-4.csv'

l = 0.395  # length of the pole, m
track_boundaries = 19.8  # boundaries of the track, cm

time, position, target_position, angle = get_data(dataset_nni_hls)

time4target, target_position = break_line_on_jump(time, target_position, threshold=0.01)

angle_sin = np.sin(angle)
angle_cos = np.cos(angle)

boundary_plus = np.full_like(time, track_boundaries)  # Create an array filled with 19.8
boundary_minus = np.full_like(time, -track_boundaries)  # Create an array filled with -19.8

#Plot the 3D curve
# Version 2: Use Plotly for an interactive plot
import plotly.graph_objects as go

# ... (Your data loading and processing here) ...

# Create traces
trace0 = go.Scatter3d(
    x=(-angle_sin * l + position) * 100.0,
    y=time,
    z=(angle_cos * l) * 100.0,
    mode='lines',
    name="Pole's Tip",
    line=dict(color='blue', width=6)
)

trace1 = go.Scatter3d(
    x=position * 100.0,
    y=time,
    z=[0]*len(time),
    mode='lines',
    name='Cart',
    line=dict(color='red', width=6)
)

trace2 = go.Scatter3d(
    x=target_position * 100.0,
    y=time4target,
    z=[0]*len(time4target),
    mode='lines',
    name='Target Position',
    line=dict(color='brown', width=6)
)

trace3 = go.Scatter3d(
    x=[track_boundaries]*len(time),
    y=time,
    z=[0]*len(time),
    mode='lines',
    name='Track Boundaries +',
    line=dict(color='black', width=6, dash='dash')
)

trace4 = go.Scatter3d(
    x=[-track_boundaries]*len(time),
    y=time,
    z=[0]*len(time),
    mode='lines',
    name='Track Boundaries -',
    line=dict(color='black', width=6, dash='dash')
)

data = [trace0, trace1, trace2, trace3, trace4]

layout = go.Layout(
    title='Interactive 3D Plot of Pole Dynamics',
    scene=dict(
        xaxis_title='Position [cm]',
        yaxis_title='Time [s]',
        zaxis_title='Height [cm]'
    )
)

fig = go.Figure(data=data, layout=layout)
fig.show()