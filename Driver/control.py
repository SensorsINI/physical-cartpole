import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(".", "Driver")))
sys.path.insert(1, os.path.abspath(os.path.join(".", "Driver", "CartPoleSimulation")))

os.chdir("Driver")

import subprocess
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "1"
from DriverFunctions.PhysicalCartPoleDriver import PhysicalCartPoleDriver
import tensorflow as tf

subprocess.call("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True, stdout=open(os.devnull, 'wb'))
subprocess.Popen("python3 DataAnalysis/plot_server.py", shell=True)


#tf.keras.backend.clear_session()
#tf.config.optimizer.set_jit(True) # Enable XLA.

#os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

#print('TF Devices:', tf.config.list_physical_devices())
#print('TF Device Placement:', tf.config.get_soft_device_placement())
#print('TF Float Type:', tf.keras.backend.floatx())

#tf.config.set_soft_device_placement(True)
#tf.debugging.set_log_device_placement(True)

PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver()
with tf.device("cpu"):
    PhysicalCartPoleDriverInstance.run()

try:
    subprocess.check_output("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True)
except:
    pass