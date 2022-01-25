import abc

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class BaseJoystick(abc.ABC):
    def get_twist_from_joy(self, joy_msg: Joy) -> Twist:
        pass

    def switch_planning_frame_indicator(self, joy_msg: Joy) -> bool:
        pass

    def toggle_gripper_indicator(self, joy_msg: Joy) -> bool:
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

    def __init__(self, control_rate) -> None:
        # helpers to do single "toggle" with button press that is not necessarily pressed for a single joy message
        self.indicator_cooldown_period = control_rate  # 1s
        self.gripper_indicator_cooldown_period = control_rate / 5  # 0.2s

        self.indicator_cooldown = self.indicator_cooldown_period
        self.gripper_indicator_cooldown = self.gripper_indicator_cooldown_period

    def get_twist_from_joy(self, joy_msg: Joy) -> Twist:

        twist = Twist()

        l1 = -1.0 * joy_msg.buttons[LogitechF310.left_l1_button] + joy_msg.buttons[LogitechF310.right_l1_button]
        l2_left = (joy_msg.axes[LogitechF310.left_l2_axis] - 1) / (-2)
        l2_right = (joy_msg.axes[LogitechF310.right_l2_axis] - 1) / (-2)
        l2 = -l2_left if l2_left > 0.01 else l2_right

        twist.linear.x = l2
        twist.linear.y = joy_msg.axes[LogitechF310.left_stick_x_axis] * -1.0
        twist.linear.z = joy_msg.axes[LogitechF310.left_stick_y_axis]

        twist.angular.x = joy_msg.axes[LogitechF310.right_stick_x_axis] * -1.0
        twist.angular.y = joy_msg.axes[LogitechF310.right_stick_y_axis]
        twist.angular.z = l1

        return twist

    def toggle_gripper_indicator(self, joy_msg: Joy) -> bool:
        if not self.gripper_indicator_cooldown and joy_msg.buttons[LogitechF310.a_button]:
            self.gripper_indicator_cooldown = self.gripper_indicator_cooldown_period
            return True
        else:
            self.gripper_indicator_cooldown = max(self.gripper_indicator_cooldown - 1, 0)
            return False

    def switch_planning_frame_indicator(self, joy_msg: Joy) -> bool:
        if not self.indicator_cooldown and joy_msg.buttons[LogitechF310.start_button]:
            self.indicator_cooldown = self.indicator_cooldown_period
            return True
        else:
            self.indicator_cooldown = max(self.indicator_cooldown - 1, 0)
            return False
