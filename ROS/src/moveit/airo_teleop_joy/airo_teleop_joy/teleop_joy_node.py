import rclpy.action
import rclpy.node
from airo_teleop_joy.joystick import LogitechF310
from control_msgs.action import GripperCommand
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

# TODO: make topic and action configurable
JOY_TOPIC_NAME = "/joy"
SERVO_TWIST_TOPIC_NAME = "/servo_node/delta_twist_cmds"

GRIPPER_ACTION_NAME = "/robotiq_gripper_controller/gripper_cmd"
DEFAULT_JOYSTICK_TYPE = "logitech_F310"
DEFAULT_EEF_PLANNING_FRAME = "tool0"
DEFAULT_BASE_PLANNING_FRAME = "base_link"

CONTROL_RATE = 250


class TeleopJoy(rclpy.node.Node):
    """
    Node that converts Joy messages into Twist commands for a robotic manipulator
    and optionally also to a GripperCommand actions to toggle a gripper between opened and closed.

    Assumes Moveit Servo is configured and activated to control the robot by subscribing to a Twist messages topic.
    Assumes a GripperController is configured and activated to control the robot using GripperCommand Action server
    (Moveit! and Ros control default gripper interface)
    """

    GRIPPER_OPEN = 1
    GRIPPER_CLOSED = 0

    def __init__(self):
        super().__init__("TeleopJoyNode")
        self.joy_subscriber = self.create_subscription(Joy, JOY_TOPIC_NAME, self.joy_subscriber_callback, 2)
        self.twist_publisher = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC_NAME, 1)

        self.control_rate = CONTROL_RATE
        self.create_timer(1.0 / self.control_rate, self.timer_callback)

        self.declare_parameter("joystick_type", DEFAULT_JOYSTICK_TYPE)
        self.declare_parameter("eef_planning_frame", DEFAULT_EEF_PLANNING_FRAME)
        self.declare_parameter("base_planning_frame", DEFAULT_BASE_PLANNING_FRAME)

        self.declare_parameter("publish_gripper", False)
        self.declare_parameter("gripper_action_server", GRIPPER_ACTION_NAME)
        self.declare_parameter("gripper_closed_position", 0.793)
        self.declare_parameter("gripper_open_position", 0.0)
        joystick_type = self.get_parameter("joystick_type").get_parameter_value().string_value
        self._logger.info(f"configuring joystick of type {joystick_type}")
        self.joystick = self.create_joystick(joystick_type, self.control_rate)

        self.base_planning_frame = self.get_parameter("base_planning_frame").get_parameter_value().string_value
        self.eef_planning_frame = self.get_parameter("eef_planning_frame").get_parameter_value().string_value
        self.current_planning_frame = self.base_planning_frame
        self.publish_gripper = self.get_parameter("publish_gripper").get_parameter_value().bool_value

        if self.publish_gripper:
            self.gripper_action_name = self.get_parameter("gripper_action_server").get_parameter_value().string_value
            self.gripper_state = TeleopJoy.GRIPPER_OPEN  # assume gripper is open on start
            self.gripper_action_client = rclpy.action.ActionClient(self, GripperCommand, self.gripper_action_name)
            self._logger.info("waiting for gripper action server to become available")
            self.gripper_action_client.server_is_ready()
            self._logger.info("gripper action server is available")

            self.gripper_closed_position = (
                self.get_parameter("gripper_closed_position").get_parameter_value().integer_value
            )
            self.gripper_open_position = (
                self.get_parameter("gripper_open_position").get_parameter_value().integer_value
            )
        self.joy_msg = None

    @staticmethod
    def create_joystick(joystick_type: str, control_rate):
        if joystick_type == "logitech_F310":
            return LogitechF310(control_rate)

        else:
            raise NotImplementedError

    def joy_subscriber_callback(self, joy_msg: Joy):
        self.joy_msg = joy_msg

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

        The idea is to be able to enforce a specific control rate for e.g. demonstrations that are afterwards used
        by a learning framework that uses the same control frequency.
        """

        if not self.joy_msg:
            return

        if self.joystick.switch_planning_frame_indicator(self.joy_msg):
            self.switch_planning_frame()

        self.publish_twist()

        if self.publish_gripper:
            self.publish_gripper_action()

    def publish_twist(self):

        twist = TwistStamped()
        twist.twist = self.joystick.get_twist_from_joy(self.joy_msg)
        twist.header.frame_id = self.current_planning_frame
        twist.header.stamp = self.get_clock().now().to_msg()

        self._logger.debug(f"publishing Twist: {twist}")
        self.twist_publisher.publish(twist)

    def publish_gripper_action(self):

        # TODO: this is perhaps not a good interface for collecting demonstrations with a gripper for RL?
        # better to create a separate gripper_servo? and allow for step updates instead of toggling open/

        # actions are pre-emptive, so if you trigger a new action the previous one is cancelled (so open/close can be +- continuous stream)

        if self.joystick.toggle_gripper_indicator(self.joy_msg):
            self._logger.info("gripper toggle triggered")
            if self.gripper_state == TeleopJoy.GRIPPER_CLOSED:
                gripper_target_state = self.gripper_open_position
            else:
                gripper_target_state = self.gripper_closed_position

            gripper_command = GripperCommand.Goal()
            gripper_command.command.position = gripper_target_state
            self.gripper_action_client.send_goal_async(gripper_command)

            self.gripper_state = 1 - self.gripper_state


def main():
    rclpy.init()
    node = TeleopJoy()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
