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
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("drive_mode", "AFRS")

        self._serial_port = self.get_parameter("serial_port").value
        self._baud_rate = self.get_parameter("baudrate").value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._drive_mode = self.get_parameter("drive_mode").value

        self._serial_connection = serial.Serial(self._serial_port, self._baud_rate)

        self._command_string = [0.0, 0.0]

        self._serial_velocity_broadcaster = SerialVelocityBroadcaster(
            self._serial_connection, self._command_string
        )

        self.create_subscription(Twist, self._cmd_vel_topic, self.twist_callback, 10)

    def twist_callback(self, twist_msg):

        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z

        if self._drive_mode == "AFRS":
            self._command_string[0] = linear_velocity
            self._command_string[1] = angular_velocity
            self._serial_velocity_broadcaster.broadcast_velocity()

        elif self._drive_mode == "differential":
            left_motor_velocity = linear_velocity - angular_velocity
            right_motor_velocity = linear_velocity + angular_velocity

            self._command_string[0] = left_motor_velocity
            self._command_string[1] = right_motor_velocity
            self._serial_velocity_broadcaster.broadcast_velocity()

        else:
            self.get_logger().info(
                "Invalid Type | Please choose between AFRS or differential only"
            )


def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
