#ifndef CONTROLLER
#define CONTROLLER

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "bur_msgs/msg/command.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <algorithm>
#include "utility_func.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace controller
{
  class PID {
  public:    
    PID (){}

    void setGains(double kp, double ki, double kd)
    {
      this->kp = kp;
      this->kd = kd;
      this->ki = ki;
    }

    double computeCommand(double error, double dt)
    {
      double currentE = error; // Error is already calculated in the function calls
      double deriv = (currentE - pastE) / dt;
      rsum += currentE * dt;
      double o = kp * currentE + kd * deriv + rsum * ki;
      pastE = currentE;
      return o;
    }
  
  private:
    double kp = 0;
    double kd = 0;
    double ki = 0;
    double pastE = 0;
    double currentE = 0;
    double rsum = 0;
  };

  class CascadedPositionVelocityController
  {
  public:
    CascadedPositionVelocityController() {}

    void setPositionGains(double kp, double ki, double kd)
    {
      position_controller.setGains(kp, ki, kd);
    }

    void setVelocityGains(double kp, double ki, double kd)
    {
      velocity_controller.setGains(kp, ki, kd);
    }

    double computeCommand(double current_pos, double target_pos, double current_vel, double target_vel, double dt)
    {
      if (this->using_external_target_vel) {
        if (abs(target_vel) < 0.1) {
          target_pos = this->position_setpoint;
        } else {
          this->position_setpoint = current_pos;
        }
      }
      double position_error = target_pos - current_pos;
      if (this->using_angles)
        position_error = angle_wrap_pi(position_error);
      double vel_target = position_controller.computeCommand(position_error, dt);

      double velocity_error;
      if (this->using_external_target_vel && abs(target_vel) >= 0.1) {
        velocity_error = target_vel - current_vel;
      } else {
        velocity_error = vel_target - current_vel;
      }
      double out = velocity_controller.computeCommand(velocity_error, dt);

      return out;
    }

    void setUsingExternalVelocityTarget(bool using_external_target_vel) {
      this->using_external_target_vel = using_external_target_vel;
    }

    void setUsingAngles(bool using_angles) {
      this->using_angles = using_angles;
    }

  private:
    PID position_controller;
    PID velocity_controller;

    double position_setpoint = 0.0;

    bool using_external_target_vel = false;
    bool using_angles = false;

  };

  class ControllerNode : public rclcpp::Node
  {
  public:
    ControllerNode();

  private:

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pubControlEffort;
    rclcpp::Subscription<bur_msgs::msg::Command>::SharedPtr state_setpoint_sub;
    rclcpp::TimerBase::SharedPtr pubTimer_;

    //  Callbacks
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    void currentCommandCallback(const bur_msgs::msg::Command::SharedPtr msg);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback;

    void set_constants();
    void publishState();

    // Controllers
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

    rclcpp::Time lastTime;

    bool active = false;
    bool using_joy = true;

    tf2::Vector3 setpoint_angle;
    tf2::Vector3 state_angle;
  };
}

#endif
