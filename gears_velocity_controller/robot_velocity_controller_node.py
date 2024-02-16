import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from SerialCommunication import SerialVelocityBroadcaster
import serial


class VelocityController(Node):

    def __init__(self):
        super().__init__("velocity_controller")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self._serial_port = self.get_parameter("serial_port").value
        self._baud_rate = self.get_parameter("baudrate").value

        self._serial_connection = serial.Serial(self._serial_port, self._baud_rate)

        self.gears_cmd_vel = Twist()

        serial_velocity_broadcaster = SerialVelocityBroadcaster(
            self._serial_connection, self.gears_cmd_vel
        )

        self.gears_cmd_vel.linear.x = 0.5
        self.gears_cmd_vel.angular.z = 1.0

        serial_velocity_broadcaster.broadcast_velocity()


def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
