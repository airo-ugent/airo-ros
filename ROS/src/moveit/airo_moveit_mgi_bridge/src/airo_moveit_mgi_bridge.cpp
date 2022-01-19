#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "airo_moveit_mgi_bridge_msgs/srv/move_to_pose.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("airo_mgi_bridge");
class MGIBridge : public rclcpp::Node
{
public:
  MGIBridge() : Node("mgi_bridge")
  {
    move_to_pose_service = this->create_service<airo_moveit_mgi_bridge_msgs::srv::MoveToPose>(
        "target_pose", [&](const std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::MoveToPose::Request> request,
                           std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::MoveToPose::Response> response) {
          move_to_pose_service_callback(request, response);
        });
    RCLCPP_INFO(LOGGER, "Initialized");
  }

  void set_mgi_node(rclcpp::Node::SharedPtr node)
  {
    mgi_node = node;
    RCLCPP_DEBUG(LOGGER, "mgi_bridge_private node reference passed.");
  }

  void
  move_to_pose_service_callback(const std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::MoveToPose::Request> request,
                                std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::MoveToPose::Response> response)
  {
    auto planning_group = "panda_arm";
    RCLCPP_INFO(LOGGER, "Received target pose for frame %s", request->frame);
    auto mgi = moveit::planning_interface::MoveGroupInterface(mgi_node, planning_group);
    RCLCPP_DEBUG(LOGGER, "Connected with MGI for group : %s", planning_group);

    mgi.setPoseTarget(request->target_pose, request->frame);
    // blocking call to MGI
    moveit::planning_interface::MoveItErrorCode error_code = mgi.move();
    RCLCPP_INFO(LOGGER, "MGI move command returned code %i", error_code.val);
    response->success = (error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

private:
  rclcpp::Service<airo_moveit_mgi_bridge_msgs::srv::MoveToPose>::SharedPtr move_to_pose_service;
  rclcpp::Node::SharedPtr mgi_node;
};

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
