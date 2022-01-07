import subprocess
from CartPole.PhysicalCartPoleDriver import PhysicalCartPoleDriver
import tensorflow as tf

try:
    output = subprocess.check_output("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True)
except:
    pass
subprocess.Popen("python3 DataAnalysis/plot_server.py", shell=True)

print('TF Devices:', tf.config.list_physical_devices())
print('TF Device Placement:', tf.config.get_soft_device_placement())
print('TF Float Type:', tf.keras.backend.floatx())

#tf.config.set_soft_device_placement(True)
#tf.debugging.set_log_device_placement(True)



PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver()
PhysicalCartPoleDriverInstance.run()

try:
    subprocess.check_output("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True)
except:
    pass