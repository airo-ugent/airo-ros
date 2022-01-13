import rclpy.node
from control_msgs.action import GripperCommand
from geometry_msgs.msg import TwistStamped
from joystick import LogitechF310
from sensor_msgs.msg import Joy

# TODO: make topics/actions configurable
JOY_TOPIC_NAME = "/joy"
SERVO_TWIST_TOPIC_NAME = "/servo_node/delta_twist_cmds"
GRIPPER_ACTION_NAME = "/gripper/gripper_cmd"

DEFAULT_JOYSTICK_TYPE = "logitech_F310"
DEFAULT_EEF_PLANNING_FRAME = "tool0"
DEFAULT_BASE_PLANNING_FRAME = "base_link"


class TeleopJoy(rclpy.node.Node):
    """
    Node that converts Joy messages into Twist commands for a robotic manipulator and optionally also to a GripperCommand actions for a gripper.
    Assumes Moveit Servo is configured and activated to control the robot by subscribing to a Twist messages topic.
    Assumes a GripperController is configured and activated to control the robot using GripperCommand Action server (Moveit! and Ros control default gripper interface)
    """

    def __init__(self):
        super().__init__("TeleopJoyNode")
        self.joy_subscriber = self.create_subscription(Joy, JOY_TOPIC_NAME, self.joy_subscriber_callback, 2)
        self.twist_publisher = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC_NAME, 1)
        self.create_timer(0.004, self.twist_publisher_timer_callback)
        self.gripper_action_client = self.create_client(GripperCommand, GRIPPER_ACTION_NAME)

        self.declare_parameter("joystick_type", DEFAULT_JOYSTICK_TYPE)
        self.declare_parameter("eef_planning_frame", DEFAULT_EEF_PLANNING_FRAME)
        self.declare_parameter("base_planning_frame", DEFAULT_BASE_PLANNING_FRAME)
        self.declare_parameter(
            "publish_gripper", False, "Bool indicating whether to publish commands to a gripper or not."
        )

        joystick_type = self.get_parameter("joystick_type").get_parameter_value().string_value
        self._logger.info(f"configuring joystick of type {joystick_type}")
        self.joystick = self.create_joystick(joystick_type)

        self.base_planning_frame = self.get_parameter("base_planning_frame").get_parameter_value().string_value
        self.eef_planning_frame = self.get_parameter("eef_planning_frame").get_parameter_value().string_value
        self.current_planning_frame = self.base_planning_frame
        self.publish_gripper = self.get_parameter("publish_gripper").get_parameter_value().string_value

        self.joy_msg = None

    @staticmethod
    def create_joystick(joystick_type: str):
        if joystick_type == "logitech_F310":
            return LogitechF310()

        else:
            raise NotImplementedError

    def joy_subscriber_callback(self, joy_msg: Joy):
        self.joy_msg = joy_msg

        if self.joystick.switch_planning_frame_indicator(joy_msg):
            self.switch_planning_frame()

    def switch_planning_frame(self):
        """
        Switch between frame used for relative motion and orientation.
        """
        if self.current_planning_frame == self.base_planning_frame:
            self.current_planning_frame = self.eef_planning_frame
        else:
            self.current_planning_frame = self.base_planning_frame
        self._logger.info(f"switched planning frame to {self.current_planning_frame}")

    def timer_callback(self):
        """
        Publish Twists and GripperCommands at a certain rate using the last available joy message.

        The idea is to be able to enforce a lower control rate for e.g. demonstrations that are afterwards used by a learning framework.
        """

        self.publish_twist()

        if self.publish_gripper:
            self.publish_gripper_action()

    def publish_twist(self):
        if not self.joy_msg:
            return

        twist = TwistStamped()
        twist.twist = self.joystick.get_twist_from_joy(self.joy_msg)
        twist.header.frame_id = self.current_planning_frame
        twist.header.stamp = self.get_clock().now().to_msg()

        self._logger.debug(f"publishing Twist: {twist}")
        self.twist_publisher.publish(twist)

    def publish_gripper_action(self):
        if not self.joy_msg:
            return

        # TODO: get last published gripper joint value and add delta provided by joy
        # won't this cause small instabilities due to sensor noise? (i.e. even is no delta is sent, this would modify the output..)
        # so maybe only send update if the buttons were pressed?
        # and maybe even just do open or close?


def main():
    rclpy.init()
    node = TeleopJoy()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
