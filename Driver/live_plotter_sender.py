from live_plotter_x_connection_handler_sender import LivePlotter_ConnectionHandlerSender


class LivePlotter_Sender:
    def __init__(self, address, use_remote=False, remote_username=None, remote_ip=None):
        if use_remote and (remote_username is None or remote_ip is None):
            raise Exception("Remote connection requires username and ip.")

        if use_remote:
            self.sender = LivePlotter_ConnectionHandlerSender(address, remote_username, remote_ip,
                                                              callback=self._update_connection_ready)
        else:
            self.sender = LivePlotter_ConnectionHandlerSender(address, callback=self._update_connection_ready)

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


