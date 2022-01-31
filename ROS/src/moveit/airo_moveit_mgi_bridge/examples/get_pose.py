#!/usr/bin/env python3

import rclpy
from airo_moveit_mgi_bridge_msgs.srv import GetFramePose

if __name__ == "__main__":
    rclpy.init()

    node = rclpy.create_node("demo")
    client = node.create_client(GetFramePose, "/moveit_mgi_bridge/get_frame_pose")

    request = GetFramePose.Request()
    request.frame = "tool0"
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)
    print(future.result())

    rclpy.shutdown()
