import sys
import os
import subprocess

# assume we start from physical-cartpole root
print('adding Driver and Driver/CartPoleSimulation to sys.path and changing cwd to Driver')
sys.path.insert(0, os.path.abspath(os.path.join(".", "Driver")))
sys.path.insert(1, os.path.abspath(os.path.join(".", "Driver", "CartPoleSimulation")))
os.chdir("Driver")

from Control_Toolkit.others.get_logger import get_logger
log = get_logger(__name__)

# Set device
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # TF: If uncommented, only uses CPU
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "0"

if "CUDA_VISIBLE_DEVICES" in os.environ and os.environ["CUDA_VISIBLE_DEVICES"] == "-1":
    log.warning('GPU disabled to allow XLA to use tf.Variables on same device, see os.environ["CUDA_VISIBLE_DEVICES"] at start of control.py')


import tensorflow as tf
from DriverFunctions.PhysicalCartPoleDriver import PhysicalCartPoleDriver
from CartPoleSimulation.CartPole import CartPole
from globals import CONTROL_PERIOD_MS

subprocess.call("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True, stdout=open(os.devnull, 'wb'))
subprocess.Popen("python3 -m DataAnalysis.plot_server", shell=True)

tf.keras.backend.clear_session()
tf.config.optimizer.set_jit(True) # Enable XLA.

log.info(f"TF Devices: {tf.config.list_physical_devices()}")
log.info(f"TF Device Placement: {tf.config.get_soft_device_placement()}")
log.info(f"TF Float Type: {tf.keras.backend.floatx()}")

#tf.config.set_soft_device_placement(True)
#tf.debugging.set_log_device_placement(True)

CartPoleInstance = CartPole()
CartPoleInstance.dt_controller = float(CONTROL_PERIOD_MS)/1000.0
PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver(CartPoleInstance)
PhysicalCartPoleDriverInstance.run()

try:
    subprocess.check_output("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True)
except:
    pass