import subprocess
from CartPole.PhysicalCartPoleDriver import PhysicalCartPoleDriver

try:
    output = subprocess.check_output("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True)
except:
    pass
subprocess.Popen("python3 DataAnalysis/plot_server.py", shell=True)

PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver()
PhysicalCartPoleDriverInstance.run()

try:
    subprocess.check_output("ps aux | grep plot_server | awk '{print $2}' | xargs kill -9 > /dev/null", shell=True)
except:
    pass