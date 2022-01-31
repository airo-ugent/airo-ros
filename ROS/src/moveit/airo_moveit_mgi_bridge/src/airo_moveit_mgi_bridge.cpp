#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "airo_moveit_mgi_bridge_msgs/srv/get_frame_pose.hpp"
#include "airo_moveit_mgi_bridge_msgs/srv/move_to_pose.hpp"
#include "airo_moveit_mgi_bridge_msgs/srv/execute_cartesian_path.hpp"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

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

    execute_cartesian_path_service = this->create_service<airo_moveit_mgi_bridge_msgs::srv::ExecuteCartesianPath>(
        "~/execute_cartesian_path",
        [&](const std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::ExecuteCartesianPath::Request> request,
            std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::ExecuteCartesianPath::Response> response) {
          execute_cartesian_path_callback(request, response);
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
    mgi.setNumPlanningAttempts(5);
    mgi.setPlanningTime(10.0);
    mgi.setWorkspace(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
    mgi.setStartStateToCurrentState();
    mgi.setGoalJointTolerance(0.02);
    // TODO(anyone): trajectories are often very inefficient w.r.t. planned trajectories from RVIZ.. Find out why!

    // blocking call to MGI
    moveit::planning_interface::MoveItErrorCode error_code = mgi.move();
    RCLCPP_INFO(LOGGER, "MGI move command returned code %i", error_code.val);
    response->success = (error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  /**
   * @brief
   * Service callback for the get pose of frame request to the Move Group Interface.
   * Frame (link) must be known to moveit (and is not limited to frames that are semantically "end effectors" as opposed
   * to what the function signature would suggest.)
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

  /**
   * @brief
   * Service callback that attempts to move through a set of cartesian waypoints (Poses) using the computeCartesianPath
   * function of the MGI. The trajectory is post-processed to scale velocities as this is not accounted for by the MGI.
   *
   */
  void execute_cartesian_path_callback(
      const std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::ExecuteCartesianPath::Request> request,
      std::shared_ptr<airo_moveit_mgi_bridge_msgs::srv::ExecuteCartesianPath::Response> response)
  {
    RCLCPP_INFO(LOGGER, "Received request to execute cartesian trajectory");
    auto mgi = moveit::planning_interface::MoveGroupInterface(mgi_node, planning_group);
    RCLCPP_DEBUG(LOGGER, "Connected with MGI for group : %s", planning_group.c_str());

    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::RobotTrajectory scaled_trajectory;
    double success_rate =
        mgi.computeCartesianPath(request->waypoints, request->eef_step, request->jump_treshold, trajectory);

    // check if planning was successful
    if (success_rate < 0.95)
    {
      RCLCPP_INFO(LOGGER, "Trajectory success was %f, which is too low, aborting.", success_rate);
      response->success = false;
    }
    else
    {
      // Post-process trajectory to limit velocity, as this is not included in the computePath call.
      //(see https://answers.ros.org/question/258127/set_max_velocity_scaling_factor-in-a-cartesian-path/)
      trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
      robot_trajectory::RobotTrajectory r_traj(mgi.getRobotModel(), planning_group);
      r_traj.setRobotTrajectoryMsg(*mgi.getCurrentState(), trajectory);
      iptp.computeTimeStamps(r_traj, request->max_velocity_scaling, request->max_acceleration_scaling);
      r_traj.getRobotTrajectoryMsg(scaled_trajectory);

      // blocking call to MGI
      mgi.execute(scaled_trajectory);
      response->success = true;
    }
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
  rclcpp::Service<airo_moveit_mgi_bridge_msgs::srv::ExecuteCartesianPath>::SharedPtr execute_cartesian_path_service;
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
