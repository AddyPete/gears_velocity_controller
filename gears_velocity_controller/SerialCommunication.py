class SerialVelocityBroadcaster:
    def __init__(self, serial_connection=None, cmd_vel=None):
        self._serial_connection = serial_connection
        self._cmd_vel = cmd_vel

    @property
    def serial_connection(self):
        return self._serial_connection

    @serial_connection.setter
    def serial_connection(self, serial_connection):
        self._serial_connection = serial_connection

    @property
    def cmd_vel(self):
        return self._cmd_vel

    @cmd_vel.setter
    def cmd_vel(self, cmd_vel):
        self._cmd_vel = cmd_vel

    def broadcast_velocity(self):

        linear_velocity = self._cmd_vel.linear.x
        angular_velocity = self._cmd_vel.angular.z

        serial_command_string = f"{linear_velocity},{angular_velocity},0,0,0,0,0,0&\n"

        self._serial_connection.write(serial_command_string.encode())


# if __name__ == "__main__":

#     serial_broadcaster = SerialVelocityBroadcaster()

#     serial_broadcaster.serial_connection = "Serial Object"
#     serial_broadcaster.cmd_vel = "cmd_vel Object"

#     print(f"{serial_broadcaster.cmd_vel} - {serial_broadcaster.serial_connection}")
