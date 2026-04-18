#ifndef CONTROLLER
#define CONTROLLER

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "bur_msgs/msg/command.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <control_toolbox/pid.hpp>
#include <algorithm>
#include "utility_func.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace controller
{
  class Pid{
  public:
    double kp = 0;
    double kd = 0;
    double ki = 0;
    double pastE = 0;
    double currentE = 0;
    double rsum = 0;
    
    Pid (){}
    Pid(double a, double b, double c, double, double, bool){
      kp = a;
      kd = b;
      ki = c;
    }

    void setGains(double a, double b, double c, double, double, bool){
      kp = a;
      kd = b;
      ki = c;
    }

    double computeCommand(double error, double dt){ //error is already calculated in the function calls
      double t = dt/1e9; // turn dt from nanoseconds -> seconds
      double currentE = error;
      double deriv = (currentE - pastE) / t;
      rsum += currentE * t;
      double o = kp * currentE + kd * deriv + rsum * ki;
      pastE = currentE;
      return o;
    }
  };

  class ControllerNode : public rclcpp::Node
  {
  public:
    ControllerNode();

  private:
    // publishers

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pubControlEffort;
    rclcpp::Subscription<bur_msgs::msg::Command>::SharedPtr state_setpoint_sub;

    rclcpp::Time prevTime;

    // implementation
    Pid linear_x;
    Pid linear_y;
    Pid linear_z;

    Pid angular_x;
    Pid angular_y;
    Pid angular_z;

    rclcpp::TimerBase::SharedPtr pubTimer_;

    // bur_rov_msgs::msg::Command command;
    geometry_msgs::msg::Pose pose_state;
    geometry_msgs::msg::Pose pose_setpoint;
    geometry_msgs::msg::Twist twist_state;
    geometry_msgs::msg::Twist twist_setpoint;

    //  callbacks
    rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback;
    void currentCommandCallback(const bur_msgs::msg::Command::SharedPtr msg);
    void set_constants();
    void publishState();

    rclcpp::Time lastTime;
    bool active = false;
    bool new_params = false;
    bool using_joy = true;
    bool use_command_target = false;
    tf2::Vector3 setpoint_angle;
    tf2::Vector3 state_angle;
    bool depth_hold = false;
    bool yaw_hold = false;
    bool roll_hold = false;
    bool pitch_hold = false;
    double yaw_hold_pos;
    double roll_hold_pos = 0;
    double pitch_hold_pos = 0;
  };
}

#endif
