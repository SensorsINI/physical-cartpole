
from SI_Toolkit.computation_library import NumpyLibrary, TensorType
import numpy as np
import math
from datetime import datetime
import yaml
import os


from scipy.interpolate import interp1d
from dataclasses import dataclass
from Control_Toolkit.Controllers import template_controller

os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"
"""
Python implementation of the theory for Sparse Envent-Based Closed Loop Control (SECLOC):
https://www.frontiersin.org/articles/10.3389/fnins.2019.00827/full
"""
class controller_secloc_lqr(template_controller):
    _computation_library = NumpyLibrary

    def configure(self):
        log_base = self.config_controller["log_base"]
        ref_period = self.config_controller["ref_period"]
        angle_dead_band = self.config_controller["angle_dead_band"]
        pos_dead_band = self.config_controller["pos_dead_band"]
        self.LQR = Event_based_LQR(log_base=log_base,ref_period=ref_period,angle_dead_band = angle_dead_band,position_dead_band = pos_dead_band)
        self.motor_map = 128
        self.interpolation = interp1d([-self.motor_map,self.motor_map], [1,-1])
        self.step_idx = 0
        self.spike_count = 0

    def step(self, s: np.ndarray, time=None, updated_attributes: "dict[str, TensorType]" = {}):
        self.update_attributes(updated_attributes)
        self.LQR.time = math.floor(time*1000)/1000
        self.spike_count += self.LQR.update(s=s,set_point=0)
        motor_signal = self.LQR.motor_signal
        if motor_signal > self.motor_map:
            motor_signal = self.motor_map
        elif motor_signal < -self.motor_map:
            motor_signal = -self.motor_map
        Q = np.float32(motor_signal)
        if Q > 1.0:
            Q = 1.0
        elif Q < -1.0:
            Q = -1.0
        self.step_idx += 1
        #print(self.spike_count)
        return Q  # normed control input in the range [-1,1]. Move cart left:- right:+ 
    
    def controller_reset(self):
        log_base = self.config_controller["log_base"]
        ref_period = self.config_controller["ref_period"]
        angle_dead_band = self.config_controller["angle_dead_band"]
        pos_dead_band = self.config_controller["pos_dead_band"]
        self.LQR = Event_based_LQR(log_base=log_base,ref_period=ref_period,angle_dead_band = angle_dead_band,position_dead_band = pos_dead_band)
        self.motor_map = 128
        self.interpolation = interp1d([-self.motor_map,self.motor_map], [1,-1])
        self.step_idx = 0
        self.spike_count = 0

    def print_help(self):
        print("\n************ SECLOC LQR ************")
        print("keystroke commands")
        print("ESC quit")
        print("k toggle control on/off (initially off)")
        print("K trigger motor position calibration")
        print("=/- increase/decrease (fine tune) angle deviation value")
        print("[/] increase/decrease position target")
        print("2/1 angle proportional gain")
        print("w/q angle integral gain")
        print("s/a angle derivative gain")
        print("z/x angle smoothing")
        print("4/3 position proportional gain")
        print("r/e position integral gain")
        print("f/d position derivative gain")
        print("p print PID parameters")
        print("l toggle logging data")
        print("S/L Save/Load param values from disk")
        print("D Toggle dance mode")
        print(",./ Turn on motor left zero right")
        print("m Toggle measurement")
        print("j Switch joystick control mode")
        print("b Print angle measurement from sensor")
        print("6 Enable/Disable live plot")
        print("5 Interrupts for histogram plot")
        print("***********************************")


@dataclass
class Event:
    time: int
    polarity: int
    change_sign: int
    n_change_base: int


class Motor_control:
    def __init__(self) -> None:
        pass

class Event_based_LQR:
    def __init__(self, log_base: float, ref_period: int, angle_dead_band: float, position_dead_band: float):
        self.ref_period = ref_period # Refractory period (in microseconds)
        self.log_base = log_base # Base for the computation of the event
        self.angle_dead_band = angle_dead_band # Dead band around the setpoint that is used to avoid reacting to sensor noise.
        self.position_dead_band = position_dead_band # Dead band around the setpoint that is used to avoid reacting to sensor noise.
        self.angle_last_shift = 0 # Last angle value stored
        self.position_last_shift = 0 # Last position value stored
        self.shift_base = 0.0
        self.has_init = False
        self.last_event_time = 0
        self.motor_signal = 0
        self.time = 0
    
    def init_sensor(self):
        self.has_init = True
        
    def update(self, s: np.ndarray, set_point: float):
        spike = 0
        angle_shift = 0 - s[0]
        position_shift = set_point - s[4]
        #print("S: "+str(s))
        if self.angle_last_shift != 0:
            ang_ratio_increase = abs(angle_shift / self.angle_last_shift)
            ang_ratio_decrease = 1/ang_ratio_increase
        elif self.position_last_shift !=0:
            pos_ratio_increase = abs(position_shift / self.position_last_shift)
            pos_ratio_decrease = 1/pos_ratio_increase
        if self.has_init == False:
            self.init_sensor()
            self.spike(x=np.array([[s[4]], [s[5]], [s[0]], [s[1]]]),x0=np.array([[set_point],[0],[0],[0]]))
            spike = 1
            self.last_event_time = self.time
            self.position_last_shift = position_shift
            self.angle_last_shift = angle_shift
        if (self.angle_last_shift == 0):
            self.angle_last_shift= angle_shift
            ang_ratio_increase = self.log_base
        if (self.position_last_shift == 0):
            self.position_last_shift= angle_shift
            pos_ratio_increase = self.log_base
        elif((abs(angle_shift) > self.angle_dead_band) or (abs(position_shift) > self.position_dead_band)) and (self.time - self.last_event_time >= self.ref_period):
            if (self.angle_last_shift != 0) and (angle_shift !=0):
                ang_ratio_increase = abs(angle_shift / self.angle_last_shift)
                ang_ratio_decrease = 1/ang_ratio_increase
                if (ang_ratio_increase >= self.log_base) or (ang_ratio_decrease >= self.log_base):
                    self.spike(x=np.array([[s[4]], [s[5]], [s[0]], [s[1]]]),x0=np.array([[set_point],[0],[0],[0]]))
                    spike = 1
                    self.last_event_time = self.time
                    self.angle_last_shift = angle_shift
            if (self.position_last_shift != 0) and (position_shift !=0):
                pos_ratio_increase = abs(position_shift / self.position_last_shift)
                pos_ratio_decrease = 1/pos_ratio_increase
                if (pos_ratio_increase >= self.log_base) or (pos_ratio_decrease >= self.log_base):
                    self.spike(x=np.array([[s[4]], [s[5]], [s[0]], [s[1]]]),x0=np.array([[set_point],[0],[0],[0]]))
                    spike = 1
                    self.last_event_time = self.time
                    self.position_last_shift = position_shift
        return spike

                
    def spike(self, x:np.ndarray, x0:np.ndarray):
        #K = np.array([-0.31622777, -3.7643385, 8.37563958, 1.31533051])
        K = np.array([-1, -4.26835109, 14.31674299, 1.61213536])
        arr=np.dot(-K, x)
        #print("x: "+str(x)+"Q: "+str(arr[0]))
        self.motor_signal = arr[0]
        
    def sign(self, x: float):
        if x > 0:
            return 1.0
        elif x < 0:
            return -1.0
        else:
            return 0

