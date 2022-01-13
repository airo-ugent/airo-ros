import rclpy.node
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy

JOY_TOPIC_NAME = "/joy"
SERVO_TWIST_TOPIC_NAME = "/servo_node/delta_twist_cmds"

DEFAULT_JOYSTICK_TYPE = "logitech_F310"
DEFAULT_EEF_PLANNING_FRAME = "tool0"
DEFAULT_BASE_PLANNING_FRAME = "base_link"


class TeleopJoy(rclpy.node.Node):
    def __init__(self):
        super().__init__("TeleopJoyNode")
        self.joy_subscriber = self.create_subscription(Joy, JOY_TOPIC_NAME, self.joy_subscriber_callback, 2)
        self.twist_publisher = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC_NAME, 1)
        self.create_timer(0.0001, self.twist_publisher_callback)

        self.declare_parameter("joystick_type", DEFAULT_JOYSTICK_TYPE)

        joystick_type = self.get_parameter("joystick_type").get_parameter_value().string_value
        self._logger.info(f"configuring joystick of type {joystick_type}")
        self.joystick = self.create_joystick(joystick_type)

        self.base_planning_frame = DEFAULT_BASE_PLANNING_FRAME
        self.eef_planning_frame = DEFAULT_EEF_PLANNING_FRAME

        self.joy_msg = None
        self.current_planning_frame = self.base_planning_frame

    @staticmethod
    def create_joystick(joystick_type: str):
        if joystick_type == "logitech_F310":
            return LogitechF310()

        else:
            raise NotImplementedError

    def joy_subscriber_callback(self, joy_msg: Joy):
        """
        create Twist message and
        """
        self.joy_msg = joy_msg
        if self.joystick.switch_planning_frame_indicator(joy_msg):
            self.switch_planning_frame()

    def switch_planning_frame(self):
        if self.current_planning_frame == self.base_planning_frame:
            self.current_planning_frame = self.eef_planning_frame
        else:
            self.current_planning_frame = self.base_planning_frame
        self._logger.info(f"switched planning frame to {self.current_planning_frame}")

    def twist_publisher_callback(self):
        if not self.joy_msg:
            return

        twist = TwistStamped()
        twist.twist = self.joystick.get_twist_from_joy(self.joy_msg)
        twist.header.frame_id = self.current_planning_frame
        twist.header.stamp = self.get_clock().now().to_msg()

        self.twist_publisher.publish(twist)


class BaseJoystick:
    def get_twist_from_joy(self, joy_msg: Joy) -> Twist:
        pass

    def get_gripper_command_from_joy(self, joy_msg: Joy) -> GripperCommand:
        pass

    def switch_planning_frame_indicator(self, joy_msg: Joy) -> bool:
        pass


class LogitechF310(BaseJoystick):
    # layout similar to PS controller, see https://www.google.com/search?q=ps+controller+layout&sxsrf=AOaemvLhxf32SK4OvUMhR2xWOvKbWBgarQ:1642062999046&tbm=isch&source=iu&ictx=1&vet=1&fir=qUbJUSLeDzhM8M%252CICiGyaESnZ-bbM%252C_%253BGbUgwbBVUubaxM%252CI8XjnV-FL26xsM%252C_%253BdWD9R__xkrjkbM%252CESA50OzRzBsJOM%252C_%253B9SPQP4Nqi5gs7M%252Ch04cpil5cL_DLM%252C_%253BucQcRhOpmSguAM%252CCQB_BReivCPchM%252C_%253Bmmw3Pbe3tAGv5M%252CtKATMfkkeg0luM%252C_%253BrRDFgwII3Zm2AM%252C2EolMoB3tbXJuM%252C_%253BpyxHernb0d6JUM%252ClhIo-WV1cXar6M%252C_%253BE2ry2SH-3QscRM%252Caqqe-ZWut5zXYM%252C_%253BLpIE557t-yBOuM%252C190yCFv3CI23rM%252C_%253B3JG0ZAi6P91ekM%252Ct4sZBoyvPeoeyM%252C_%253B1tYytQ1up2dPaM%252CGabD2IUhCXFXLM%252C_&usg=AI4_-kS_wnzNIr9a4ZWxaqjpn0nX1DlbXg&sa=X&ved=2ahUKEwjByYauqa71AhUNyqQKHfk7BAQQ9QF6BAgIEAE&biw=1920&bih=975&dpr=1#imgrc=pyxHernb0d6JUM
    left_stick_x_axis = 0
    left_stick_y_axis = 1
    left_l2_axis = 2
    right_stick_x_axis = 3
    right_stick_y_axis = 4
    right_l2_axis = 5
    dpad_x_axis = 6
    dpad_y_axis = 7

    a_button = 0
    b_button = 1
    x_button = 2
    y_button = 3
    left_l1_button = 4
    right_l1_button = 5
    back_button = 6
    start_button = 7

    # helper to do single switch with button press that is not necessarily pressed for single joy message
    indicator_cooldown = 5

    def get_twist_from_joy(self, joy_msg: Joy) -> Twist:

        twist = Twist()

        l2_is_angular = joy_msg.buttons[LogitechF310.left_l1_button] or joy_msg.buttons[LogitechF310.right_l1_button]
        l2_left = (joy_msg.axes[LogitechF310.left_l2_axis] - 1) / (-2)
        l2_right = (joy_msg.axes[LogitechF310.right_l2_axis] - 1) / (-2)
        l2 = -l2_left if l2_left > 0.01 else l2_right

        twist.linear.x = 0.0 if l2_is_angular else l2
        twist.linear.y = joy_msg.axes[LogitechF310.left_stick_x_axis] * -1.0
        twist.linear.z = joy_msg.axes[LogitechF310.left_stick_y_axis]

        twist.angular.x = joy_msg.axes[LogitechF310.right_stick_x_axis] * -1.0
        twist.angular.y = joy_msg.axes[LogitechF310.right_stick_y_axis]
        twist.angular.z = l2 if l2_is_angular else 0.0

        return twist

    def switch_planning_frame_indicator(self, joy_msg: Joy) -> bool:
        if not self.indicator_cooldown and joy_msg.buttons[LogitechF310.start_button]:
            self.indicator_cooldown = 5
            return True
        else:
            self.indicator_cooldown = max(self.indicator_cooldown - 1, 0)
            return False


def main():
    rclpy.init()
    node = TeleopJoy()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
