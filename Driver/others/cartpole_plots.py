import os, sys
print(f'cwd is {os.getcwd()}')
print(f'path is {sys.path}')

#%%
import easygui
import numpy as np
import matplotlib.pyplot as plt
#%%
from CartPoleSimulation.CartPole.load import load_csv_recording
#%%
import easygui
fp=easygui.fileopenbox(default='Driver/ExperimentRecordings/*.csv',filetypes=['*.csv'])
if not fp is None:
    p=load_csv_recording(fp) # cartpole with logged states and state predictions at next timestep, to measure model mismatch

#%%

import matplotlib
matplotlib.rcParams.update({'font.size': 12})
t=p.time-p.time[0]
tlim=25
aspect=5
fs=8
#%%


plt.clf()
plt.figure(figsize=(10,3))
plt.subplot(2,1,1)
# plt.gca().set_aspect(aspect)
plt.plot(t[t<tlim],p.position[t<tlim], t[t<tlim],p.predict_position[t<tlim], alpha=.5)
plt.xlabel('time(s)')
plt.ylabel('cart pos (m)')
plt.legend(['measured','predicted'])
plt.subplot(2,1,2)
# plt.gca().set_aspect(aspect)
plt.plot(t[t<tlim],p.angle[t<tlim]*180/np.pi, t[t<tlim],p.predict_angle[t<tlim]*180/np.pi, alpha=.5)
plt.xlabel('time(s)')
plt.ylabel('cart angle (deg)')
plt.legend(['measured','predicted'])


import matplotlib
matplotlib.rcParams.update({'font.size': 12})
tlim=25
#%%

t=p.time-p.time[0]
aspect=5
fs=8
pl=.398 # m

# compute pole tip position
cpx=p.position # cart position
cpy=np.zeros_like(cpx)
ptx=cpx+pl*np.sin(p.angle) # pole tip x
pty=pl*np.cos(p.angle) # pole tip y
#%%

ax = plt.figure().add_subplot(projection='3d')

ax.plot(xs=cpx[t<tlim],ys=cpy[t<tlim], zs=t[t<tlim], zdir='y')
ax.plot(xs=ptx[t<tlim],ys=pty[t<tlim], zs=t[t<tlim], zdir='y')
plt.show()
