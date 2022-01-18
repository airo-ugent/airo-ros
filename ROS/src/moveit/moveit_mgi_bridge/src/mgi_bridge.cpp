#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class MGIBridge : public rclcpp::Node
{
public:
  MGIBridge() : Node("mgi_bridge")
  {
    demo_subscriber = this->create_subscription<std_msgs::msg::String>(
        "test", 10, [&](const std_msgs::msg::String& msg) { demo_callback(msg); });

    RCLCPP_INFO(this->get_logger(), "initialized");
  }

  void set_mgi_node(rclcpp::Node::SharedPtr node)
  {
    mgi_node = node;
    RCLCPP_INFO(this->get_logger(), "mgi_node reference passed.");
  }

  void demo_callback(const std_msgs::msg::String& str)
  {
    RCLCPP_INFO(LOGGER, "received message on the callback");
    auto mgi = moveit::planning_interface::MoveGroupInterface(mgi_node, "ur_manipulator");
    RCLCPP_INFO(this->get_logger(), "Got MGI");

    // auto joints  = mgi.getJointNames();
    // auto pose  = geometry_msgs::msg::Pose();
    // pose.position. x = 0.3;
    // pose.position.y = 0.2;
    // pose.position.z = 0.3;
    // pose.orientation.w = 1.0;

    // mgi.setPoseTarget(pose);
    // RCLCPP_INFO(this->get_logger(), "set target pose");

    // mgi.move();
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr demo_subscriber;
  rclcpp::Node::SharedPtr mgi_node;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
  auto mgi = std::make_shared<MGIBridge>();
  mgi->set_mgi_node(move_group_node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.add_node(mgi);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
