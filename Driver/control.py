import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(".", "Driver")))
sys.path.insert(1, os.path.abspath(os.path.join(".", "Driver", "CartPoleSimulation")))

# Set device
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # TF: If uncommented, only uses CPU
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "1"

os.chdir("Driver")

import tensorflow as tf
from DriverFunctions.PhysicalCartPoleDriver import PhysicalCartPoleDriver
from CartPoleSimulation.CartPole import CartPole
from globals import CONTROL_PERIOD_MS

tf.keras.backend.clear_session()
tf.config.optimizer.set_jit(True) # Enable XLA.

print("TF Devices:", tf.config.list_physical_devices())
print("TF Device Placement:", tf.config.get_soft_device_placement())
print("TF Float Type:", tf.keras.backend.floatx())

CartPoleInstance = CartPole()
CartPoleInstance.dt_controller = float(CONTROL_PERIOD_MS)/1000.0
PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver(CartPoleInstance)
PhysicalCartPoleDriverInstance.run()
