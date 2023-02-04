import subprocess
from CartPole.PhysicalCartPoleDriver import PhysicalCartPoleDriver

try:
    output = subprocess.check_output("ps aux | grep plot_server | grep -v grep | awk '{print $2}' | xargs kill", shell=True)
except:
    pass
subprocess.Popen("python3 DataAnalysis/plot_server.py", shell=True)

PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver()
PhysicalCartPoleDriverInstance.run()

subprocess.check_output("ps aux | grep plot_server | grep -v grep | awk '{print $2}' | xargs kill", shell=True)