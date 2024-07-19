import subprocess
from multiprocessing.connection import Client
import time
import pandas as pd

USE_REMOTE = False

REMOTE_USERNAME = 'marcinpaluch'
REMOTE_IP = '192.168.194.233'
DEFAULT_ADDRESS = ('localhost', 6000)


class LivePlotter_Sender:
    def __init__(self, address, remote_username=None, remote_ip=None):
        self.address = address
        self.remote_username = remote_username
        self.remote_ip = remote_ip

        self.ssh_process = None
        if remote_username and remote_ip:
            self.use_remote = True
        else:
            self.use_remote = False

        self.connection_ready = False

        self.connection = None

    def establish_ssh_tunnel(self):
        # Define the SSH command to establish the tunnel
        ssh_command = [
            'ssh',
            '-L', f"{self.address[1]}:{self.address[0]}:{self.address[1]}",  # Local port 6000 to remote port 6000
            f"{self.remote_username}@{self.remote_ip}"  # Replace with your username and remote host
        ]

        # Start the SSH tunnel as a subprocess
        process = subprocess.Popen(ssh_command)
        return process

    def connect(self):
        if self.use_remote:
            self.ssh_process = self.establish_ssh_tunnel()
            time.sleep(5)
        else:
            time.sleep(3)
        self.connection = Client(self.address)
        self.connection_ready = True

    def send(self, data):
        pass

    def close(self):
        self.connection.close()  # Close the connection

        if self.use_remote:
            # Terminate the SSH tunnel
            self.ssh_process.terminate()
            self.ssh_process.wait()  # Wait for the SSH process to terminate


def main():
    if USE_REMOTE:
        sender = LivePlotter_Sender(DEFAULT_ADDRESS, REMOTE_USERNAME, REMOTE_IP)
    else:
        sender = LivePlotter_Sender(DEFAULT_ADDRESS)
    sender.connect()

    while not sender.connection_ready:
        time.sleep(0.1)

    path = './ExperimentRecordings/CPP_mpc__2024-07-01_00-52-52.csv'
    # path = '../../../ExperimentRecordings/CPP_mpc__2024-07-01_00-52-52.csv'
    df = pd.read_csv(path, comment='#')


    df = df[['angle', 'angleD', 'position', 'positionD']]

    try:
        # Send the header
        header = df.columns.tolist()
        sender.connection.send(header)
        time.sleep(0.1)

        # Send data line by line
        for _, row in df.iterrows():
            sender.connection.send(row.values)
            time.sleep(0.1)  # Wait for a short time before sending the next row

        # Optionally, send a 'complete' message to indicate end of data
        sender.connection.send('complete')

    finally:
        sender.close()


if __name__ == '__main__':
    main()
