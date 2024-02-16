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
        self.declare_parameter("cmd_vel_topic")

        self._serial_port = self.get_parameter("serial_port").value
        self._baud_rate = self.get_parameter("baudrate").value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self._serial_connection = serial.Serial(self._serial_port, self._baud_rate)

        self._gears_cmd_vel = Twist()

        self._serial_velocity_broadcaster = SerialVelocityBroadcaster(
            self._serial_connection, self._gears_cmd_vel
        )

        self.create_subscription(Twist, self._cmd_vel_topic, self.twist_callback, 10)

    def twist_callback(self, twist_msg):
        self._gears_cmd_vel.linear.x = twist_msg.linear.x
        self._gears_cmd_vel.angular.z = twist_msg.angular.z
        self._serial_velocity_broadcaster.broadcast_velocity()


def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
