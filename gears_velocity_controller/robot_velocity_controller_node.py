import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from SerialCommunication import SerialVelocityBroadcaster
import serial

ROTATION_INCREMENT = 0.0075
MAX_STEER = 0.3


class VelocityController(Node):

    def __init__(self):
        super().__init__("velocity_controller")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("drive_mode", "4WS")

        self._serial_port = self.get_parameter("serial_port").value
        self._baud_rate = self.get_parameter("baudrate").value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._drive_mode = self.get_parameter("drive_mode").value

        self._serial_connection = serial.Serial(self._serial_port, self._baud_rate)

        self._command_string = [0.0, 0.0, 0]

        self._mode = 0  # 4WS

        self._serial_velocity_broadcaster = SerialVelocityBroadcaster(
            self._serial_connection, self._command_string, self._drive_mode
        )

        self._mode_publisher = self.create_publisher(String, "/mode", 10)

        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self.twist_callback, 10)
        self.create_subscription(String, "/mode", self.mode_callback, 10)

    def twist_callback(self, twist_msg):

        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z

        self.get_logger().info(f"Angular Velocity: {angular_velocity}")
        self.get_logger().info(f"Drive Mode: {self._drive_mode}")

        if self._drive_mode == "4WS":
            self._command_string[0] = linear_velocity

            if angular_velocity > 0:

                check_val = self._command_string[1]
                check_val -= ROTATION_INCREMENT
                if check_val >= -MAX_STEER:
                    self._command_string[1] -= ROTATION_INCREMENT
            elif angular_velocity < 0:

                check_val = self._command_string[1]
                check_val += ROTATION_INCREMENT
                if check_val <= MAX_STEER:
                    self._command_string[1] += ROTATION_INCREMENT

            else:
                self._command_string[1] = 0

            self.get_logger().info(f"Broadcast")

            self._serial_velocity_broadcaster.broadcast_velocity()

        elif self._drive_mode == "differential":
            left_motor_velocity = linear_velocity - angular_velocity
            right_motor_velocity = linear_velocity + angular_velocity

            self._command_string[0] = left_motor_velocity
            self._command_string[1] = right_motor_velocity
            self._serial_velocity_broadcaster.broadcast_velocity()

        else:
            self.get_logger().info(
                "Invalid Type | Please choose between 4WS or differential only"
            )

    def joy_callback(self, joy_msg):

        if joy_msg.axes[7] == 1:
            self.publish_mode("4ws")

        # Check the 7th axis (index 6)
        elif joy_msg.axes[6] == -1:
            self.publish_mode("spin")

    def publish_mode(self, mode):

        self._mode_publisher.publish(String(data=mode))
        self.get_logger().info(f'Publishing mode: "{mode}"')

    def mode_callback(self, msg):
        # This function is called when a new message is received on the /mode topic
        mode = msg.data  # Extract the data (string) from the message

        if mode.lower() == "4ws":
            self._mode = 0
        elif mode.lower() == "spin":
            self._mode = 1

        self._command_string[0] = 0
        self._command_string[1] = 0
        self._command_string[2] = self._mode
        self._serial_velocity_broadcaster.broadcast_velocity()
        self.get_logger().info(f'Received mode: "{mode} | VAL: {self._mode}"')


def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
