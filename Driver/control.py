import subprocess
from CartPole.PhysicalCartPoleDriver import PhysicalCartPoleDriver

subprocess.check_output("ps aux | grep plot_server | grep -v grep | awk '{print $2}' | xargs kill", shell=True)
subprocess.Popen("python3 DataAnalysis/plot_server.py", shell=True)

PhysicalCartPoleDriverInstance = PhysicalCartPoleDriver()
PhysicalCartPoleDriverInstance.run()

subprocess.check_output("ps aux | grep plot_server | grep -v grep | awk '{print $2}' | xargs kill", shell=True)