import subprocess
from multiprocessing.connection import Client
import time
import pandas as pd

USE_REMOTE = False
REMOTE_USERNAME = 'marcinpaluch'
REMOTE_IP = '192.168.194.233'


def establish_ssh_tunnel():
    # Define the SSH command to establish the tunnel
    ssh_command = [
        'ssh',
        '-L', '6000:localhost:6000',  # Local port 6000 to remote port 6000
        f"{REMOTE_USERNAME}@{REMOTE_IP}"  # Replace with your username and remote host
    ]

    # Start the SSH tunnel as a subprocess
    process = subprocess.Popen(ssh_command)
    return process


def main():
    if USE_REMOTE
        # Establish SSH tunnel
        ssh_process = establish_ssh_tunnel()
        time.sleep(5)  # Wait for the tunnel to be established

    # Create a sample DataFrame
    time.sleep(3)
    path = './ExperimentRecordings/CPP_mpc__2024-07-01_00-52-52.csv'
    df = pd.read_csv(path, comment='#')


    df = df[['angle', 'angleD', 'position', 'positionD']]

    # Use the server's local IP address
    address = ('localhost', 6000)  # Replace with the server's local IP address if necessary
    conn = Client(address)

    try:
        # Send the header
        header = df.columns.tolist()
        conn.send(header)
        time.sleep(0.1)

        # Send data line by line
        for _, row in df.iterrows():
            conn.send(row.values)
            time.sleep(0.02)  # Wait for a short time before sending the next row

        # Optionally, send a 'complete' message to indicate end of data
        conn.send('complete')

    finally:
        conn.close()  # Close the connection

        if USE_REMOTE:
            # Terminate the SSH tunnel
            ssh_process.terminate()
            ssh_process.wait()  # Wait for the SSH process to terminate


if __name__ == '__main__':
    main()
