#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class TransformStreamer : public rclcpp::Node
{
public:
  TransformStreamer() : Node("tf2_transform_streamer")
  {
    // declare and get parameters
    this->declare_parameter<std::string>("reference_frame", "base_link");
    this->declare_parameter<std::string>("frame", "tool0");

    this->get_parameter<std::string>("frame", frame);
    this->get_parameter<std::string>("reference_frame", reference_frame);

    // create tf2 listener
    RCLCPP_INFO(this->get_logger(), "Initializing tf2 buffer and listener");
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // create timer
    timer = this->create_wall_timer(200ms, [&]() { timer_callback(); });
    RCLCPP_INFO(this->get_logger(), "Transform publisher initialized for frame %s expressed in frame %s", frame.c_str(),
                reference_frame.c_str());
  }

private:
  void timer_callback()
  {
    try
    {
      // get frame pose wrt reference_frame.
      auto transformStamped = tf_buffer->lookupTransform(reference_frame, frame, tf2::TimePointZero);
      auto transform = transformStamped.transform;
      RCLCPP_INFO(this->get_logger(), "Transform from %s to %s \n translation:  (%f,%f,%f), rotation: (%f,%f, %f, %f) ",
                  reference_frame.c_str(), frame.c_str(), transform.translation.x, transform.translation.y,
                  transform.translation.z, transform.rotation.x, transform.rotation.y, transform.rotation.z,
                  transform.rotation.w);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s. Shutting down.", frame.c_str(),
                  reference_frame.c_str(), ex.what());
      rclcpp::shutdown();
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener;
  rclcpp::TimerBase::SharedPtr timer;
  std::string reference_frame;
  std::string frame;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformStreamer>());
}
