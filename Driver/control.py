import sys
import os
import subprocess

sys.path.insert(0, os.path.abspath(os.path.join(".", "Driver")))
sys.path.insert(1, os.path.abspath(os.path.join(".", "Driver", "CartPoleSimulation")))

# Set device
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # TF: If uncommented, only uses CPU
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "1"

os.chdir("Driver")

import tensorflow as tf
from DriverFunctions.PhysicalCartPoleDriver import PhysicalCartPoleDriver

subprocess.call("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True, stdout=open(os.devnull, 'wb'))
subprocess.Popen("python3 -m DataAnalysis.plot_server", shell=True)

tf.keras.backend.clear_session()
tf.config.optimizer.set_jit(True) # Enable XLA.

print("TF Devices:", tf.config.list_physical_devices())
print("TF Device Placement:", tf.config.get_soft_device_placement())
print("TF Float Type:", tf.keras.backend.floatx())

#tf.config.set_soft_device_placement(True)
#tf.debugging.set_log_device_placement(True)

PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver()
PhysicalCartPoleDriverInstance.run()

try:
    subprocess.check_output("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True)
except:
    pass