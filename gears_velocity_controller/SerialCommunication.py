class SerialVelocityBroadcaster:
    def __init__(self, serial_connection=None, cmd_string=None, drive_mode=None):
        self._serial_connection = serial_connection
        self._cmd_string = cmd_string
        self._drive_mode = drive_mode

    @property
    def cmd_string(self):
        return self._cmd_string

    @cmd_string.setter
    def cmd_string(self, cmd_string):
        self._cmd_string = cmd_string

    def broadcast_velocity(self):

        if self._drive_mode == "4WS":

            serial_command_string = (
                f"{self._cmd_string[0]},{self._cmd_string[1]},0,0,0,0,0,0&\n"
            )

        else:
            serial_command_string = f"{self._cmd_string[0]},{self._cmd_string[1]}&\n"

        self._serial_connection.write(serial_command_string.encode())
