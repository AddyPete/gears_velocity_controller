class SerialVelocityBroadcaster:
    def __init__(self, serial_connection=None, cmd_string=None):
        self._serial_connection = serial_connection
        self._cmd_string = cmd_string

    @property
    def serial_connection(self):
        return self._serial_connection

    @serial_connection.setter
    def serial_connection(self, serial_connection):
        self._serial_connection = serial_connection

    @property
    def cmd_string(self):
        return self._cmd_string

    @cmd_string.setter
    def cmd_string(self, cmd_string):
        self._cmd_string = cmd_string

    def broadcast_velocity(self):

        linear_velocity = self._cmd_string[0]
        angular_velocity = self._cmd_string[1]

        serial_command_string = f"{linear_velocity},{angular_velocity},0,0,0,0,0,0&\n"

        self._serial_connection.write(serial_command_string.encode())
