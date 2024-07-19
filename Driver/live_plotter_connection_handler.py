from multiprocessing.connection import Listener
import threading

class ConnectionHandler:
    def __init__(self, address=('0.0.0.0', 6000)):
        self.listener = Listener(address)
        self.connection = None

    def open_connection(self):
        self.connection_thread = threading.Thread(target=self.accept_connection)
        self.connection_thread.daemon = True  # Allow thread to exit when main program exits
        self.connection_thread.start()

    def accept_connection(self):
        while True:
            # print('Waiting for connection...')
            self.connection = self.listener.accept()
            print(f'Connected to: {self.listener.last_accepted}')

    def poll_connection(self, timeout=0.01):
        if self.connection is not None:
            try:
                while self.connection.poll(timeout):
                    buffer = self.connection.recv()
                    yield buffer
            except EOFError:
                print('Connection closed')
                self.connection = None
        else:
            self.open_connection()
