#ifndef CONTROLLER_NODE
#define CONTROLLER_NODE

#include "bur_msgs/msg/command.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <vector>

#include "controller.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:

  // callbacks
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback;

  void currentCommandCallback(const bur_msgs::msg::Command::SharedPtr msg);
  void set_constants();
  void publishState();

  // ROS infra
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pubControlEffort;
  rclcpp::Subscription<bur_msgs::msg::Command>::SharedPtr state_setpoint_sub;
  rclcpp::TimerBase::SharedPtr pubTimer_;

  // controllers
  CascadedPositionVelocityController linear_x;
  CascadedPositionVelocityController linear_y;
  CascadedPositionVelocityController linear_z;

  CascadedPositionVelocityController angular_x;
  CascadedPositionVelocityController angular_y;
  CascadedPositionVelocityController angular_z;

  geometry_msgs::msg::Pose pose_state;
  geometry_msgs::msg::Pose pose_setpoint;
  geometry_msgs::msg::Twist twist_state;
  geometry_msgs::msg::Twist twist_setpoint;

  // RPY current & target orientations
  tf2::Vector3 setpoint_angle;
  tf2::Vector3 state_angle;

  rclcpp::Time lastTime;

  bool active = false;
  bool using_joy = true;
};

#endif
