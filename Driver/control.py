import sys
import os

sys.path.insert(0, os.path.abspath("."))  # repo root, for 'from Driver.DriverFunctions...' imports
sys.path.insert(1, os.path.abspath(os.path.join(".", "Driver")))
sys.path.insert(2, os.path.abspath(os.path.join(".", "Driver", "CartPoleSimulation")))
sys.path.insert(3, os.path.abspath(os.path.join(".", "Driver", "CartPoleSimulation", "SI_Toolkit", "src")))


# Pinning is driven solely by globals.CONTROL_CPU_AFFINITY: set it for single-threaded
# optimizers (rpgd), clear it for parallel ones (rpgd-c). Must run before TF imports.
from globals import CONTROL_CPU_AFFINITY, CONTROL_CUDA_VISIBLE_DEVICES
from DriverFunctions.cpu_affinity import configure_control_cpu_policy

if CONTROL_CUDA_VISIBLE_DEVICES is not None:
    os.environ["CUDA_VISIBLE_DEVICES"] = str(CONTROL_CUDA_VISIBLE_DEVICES)
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "1"

_policy = configure_control_cpu_policy(CONTROL_CPU_AFFINITY)

os.chdir("Driver")

from DriverFunctions.PhysicalCartPoleDriver import PhysicalCartPoleDriver
from CartPoleSimulation.CartPole import CartPole
from globals import CONTROLLER_APPLY_WINDOW_MS

print("TF threads (env):", _policy["tf_threads"])
print("XLA Flags:", _policy["xla_flags"])

CartPoleInstance = CartPole()
CartPoleInstance.dt_controller = float(CONTROLLER_APPLY_WINDOW_MS) / 1000.0
PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver(CartPoleInstance)
print("\nPhysical CartPole Driver created!\n")
PhysicalCartPoleDriverInstance.run()
