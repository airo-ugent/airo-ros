"""
Example code for executing a cartesian path using the MGI Bridge

execute by calling `python3 execute_cartesian_path.py` from within this folder (not discoverable by ROS)
"""
import rclpy
from airo_moveit_mgi_bridge_msgs.srv import ExecuteCartesianPath
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rclpy.init()

    node = rclpy.create_node("demo")
    client = node.create_client(ExecuteCartesianPath, "/moveit_mgi_bridge/execute_cartesian_path")

    request = ExecuteCartesianPath.Request()
    p1 = Pose()
    p1.position.x = 0.4
    p1.position.z = 0.1
    p1.orientation.x = 1.0
    p1.orientation.w = 0.0

    p2 = Pose()
    p2.position.x = 0.35
    p2.position.z = 0.1
    p2.orientation.x = 1.0
    p2.orientation.w = 0.0

    p3 = Pose()
    p3.position.y = 0.1
    p3.position.x = 0.35
    p3.position.z = 0.1
    p3.orientation.x = 1.0
    p3.orientation.w = 0.0
    waypoints = [p1, p2, p3]

    request.waypoints = waypoints
    request.jump_treshold = 3.0
    request.eef_step = 0.01
    request.max_velocity_scaling = 0.2
    request.max_acceleration_scaling = 0.2

    # send request and wait for return
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    print(future.result())

    rclpy.shutdown()
