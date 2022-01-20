#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "airo_moveit_mgi_bridge_msgs/srv/get_frame_pose.hpp"
#include "airo_moveit_mgi_bridge_msgs/srv/move_to_pose.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mgi_bridge.server");

/**
 * @brief
 * Class that exposes parts of the Moveit Group Interface over ROS services, which allows to address them from other
 * clients or the CLI.
 */
class MGIBridge : public rclcpp::Node
{
public:
  MGIBridge() : Node("mgi_bridge")
  {
    configure_parameters();

    move_to_pose_service = this->create_service<airo_moveit_mgi_bridge_msgs::srv::MoveToPose>(
        "~/move_to_pose", [&](const std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::MoveToPose::Request> request,
                              std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::MoveToPose::Response> response) {
          move_to_pose_service_callback(request, response);
        });

    get_frame_pose_service = this->create_service<airo_moveit_mgi_bridge_msgs::srv::GetFramePose>(
        "~/get_frame_pose", [&](const std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::GetFramePose::Request> request,
                                std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::GetFramePose::Response> response) {
          get_frame_pose_service_callback(request, response);
        });

    RCLCPP_INFO(LOGGER, "Initialized");
  }

  /**
   * @brief
   * sets a reference to the node that should be used for the Move Group Interface constructor.
   * This is a different node for technical reasons, (even though I think it should work with the same node if you use a
   * Multithreaded executor..)
   */
  void set_mgi_node(rclcpp::Node::SharedPtr node)
  {
    mgi_node = node;
    RCLCPP_DEBUG(LOGGER, "mgi_bridge_private node reference passed.");
  }

private:
  /**
   * @brief
   * Service callback that attempts to move the requested frame to the requested pose using the Move Group Interface.
   * This uses the default planning configuration that was set in the Move Group Node.
   * In order to succeed, the frame needs to be known by Moveit, implying that it needs to be in the URDF/SRDF
   * descriptions that were loaded to the MGI and the Move Group nodes in their respective launch files.
   */
  void
  move_to_pose_service_callback(const std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::MoveToPose::Request> request,
                                std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::MoveToPose::Response> response)
  {
    RCLCPP_INFO(LOGGER, "Received target pose for frame %s", request->frame.c_str());
    auto mgi = moveit::planning_interface::MoveGroupInterface(mgi_node, planning_group);
    RCLCPP_DEBUG(LOGGER, "Connected with MGI for group : %s", planning_group.c_str());

    mgi.setPoseTarget(request->target_pose, request->frame);
    // blocking call to MGI
    moveit::planning_interface::MoveItErrorCode error_code = mgi.move();
    RCLCPP_INFO(LOGGER, "MGI move command returned code %i", error_code.val);
    response->success = (error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  /**
   * @brief
   * Service callback for the get pose of frame request to the Move Group Interface.
   * Frame (link) must be known to moveit (and is not limited to frames that are semantically "end effectors as opposed
   * to what the")
   */
  void
  get_frame_pose_service_callback(const std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::GetFramePose::Request> request,
                                  std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::GetFramePose::Response> response)
  {
    RCLCPP_INFO(LOGGER, "Received request for pose of frame %s", request->frame.c_str());
    auto mgi = moveit::planning_interface::MoveGroupInterface(mgi_node, planning_group);
    RCLCPP_DEBUG(LOGGER, "Connected with MGI for group : %s", planning_group.c_str());

    response->pose = mgi.getCurrentPose(request->frame).pose;
  }

  void configure_parameters()
  {
    this->declare_parameter<std::string>("planning_group");
    this->get_parameter<std::string>("planning_group", planning_group);
    RCLCPP_INFO(LOGGER, " Planning group is set to %s", planning_group.c_str());
  }

  rclcpp::Node::SharedPtr mgi_node;
  std::string planning_group;
  rclcpp::Service<airo_moveit_mgi_bridge_msgs::srv::MoveToPose>::SharedPtr move_to_pose_service;
  rclcpp::Service<airo_moveit_mgi_bridge_msgs::srv::GetFramePose>::SharedPtr get_frame_pose_service;
};

/**
 * @brief
 * Starts up the MGI server and an additional node, using a multithreaded executor.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto move_group_node = rclcpp::Node::make_shared("mgi_bridge_private");
  auto mgi = std::make_shared<MGIBridge>();
  mgi->set_mgi_node(move_group_node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.add_node(mgi);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
