import time
import threading
import subprocess
from multiprocessing.connection import Client


class LivePlotter_Sender:
    def __init__(self, address, use_remote=False, remote_username=None, remote_ip=None):
        if use_remote and (remote_username is None or remote_ip is None):
            raise Exception("Remote connection requires username and ip.")

        if use_remote:
            self.sender = LivePlotter_Sender_ConnectionHandler(address, remote_username, remote_ip,
                                                               callback=self._update_connection_ready)
        else:
            self.sender = LivePlotter_Sender_ConnectionHandler(address, callback=self._update_connection_ready)

        self.connection_ready = False
        self.headers_sent = False

    def connect(self):
        self.sender.connect()

    def _update_connection_ready(self, status):
        """Callback to update the connection_ready flag."""
        self.connection_ready = status

    def send_headers(self, header):
        if self.sender.connection_ready:
            self.sender.send(header)
            self.headers_sent = True
        else:
            raise Exception("Attempted sending headers but connection not established yet.")

    def send_data(self, data):
        if self.sender.connection_ready:
            if self.headers_sent:
                self.sender.send(data)
            else:
                raise Exception("Attempted sending data but headers not sent yet.")
        else:
            raise Exception("Attempted sending data but connection not established yet.")

    def send_save(self):
        if self.sender.connection_ready:
            self.sender.send('save')
        else:
            raise Exception("Attempted sending 'save' message but connection not established yet.")

    def send_reset(self):
        if self.sender.connection_ready:
            self.sender.send('reset')
        else:
            raise Exception("Attempted sending 'reset' message but connection not established yet.")

    def send_complete(self):
        if self.sender.connection_ready:
            self.sender.send('complete')
        else:
            raise Exception("Attempted sending 'complete' message but connection not established yet.")

    def close(self):
        if self.sender.connection_ready:
            self.send_complete()
        self.sender.close()


class LivePlotter_Sender_ConnectionHandler:
    def __init__(self, address, remote_username=None, remote_ip=None, callback=None):
        self.address = address
        self.remote_username = remote_username
        self.remote_ip = remote_ip

        self.ssh_process = None
        if remote_username and remote_ip:
            self.use_remote = True
        else:
            self.use_remote = False

        self._connection_ready = False

        self.connection = None
        self.callback = callback  # Callback to notify when connection is ready

    def establish_ssh_tunnel(self):
        # Define the SSH command to establish the tunnel
        ssh_command = [
            'ssh',
            '-L', f"{self.address[1]}:{self.address[0]}:{self.address[1]}",  # Local port to remote port
            f"{self.remote_username}@{self.remote_ip}"  # Username and remote host
        ]

        # Start the SSH tunnel as a subprocess
        process = subprocess.Popen(ssh_command)
        return process

    def connect(self):
        thread = threading.Thread(target=self._connect)
        thread.start()

    def _connect(self):
        if self.use_remote:
            self.ssh_process = self.establish_ssh_tunnel()
            time.sleep(5)  # Simulate waiting for SSH tunnel to establish
        else:
            time.sleep(3)  # Simulate waiting for local setup
        self.connection = Client(self.address)
        self.connection_ready = True

    @property
    def connection_ready(self):
        return self._connection_ready

    @connection_ready.setter
    def connection_ready(self, value):
        self._connection_ready = value
        if value and self.callback:
            self.callback(value)  # Call the callback when connection is ready

    def send(self, data):
        if not self.connection_ready:
            raise Exception("Connection not established yet.")
        self.connection.send(data)

    def close(self):
        if self.connection:
            self.connection.close()  # Close the connection
            self.connection_ready = False

        if self.use_remote and self.ssh_process:
            # Terminate the SSH tunnel
            self.ssh_process.terminate()
            self.ssh_process.wait()  # Wait for the SSH process to terminate