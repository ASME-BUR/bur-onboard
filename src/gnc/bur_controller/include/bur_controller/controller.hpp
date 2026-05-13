#ifndef CONTROLLER
#define CONTROLLER

#include "utility_func.hpp"


class PID
{
public:    
  PID(){}

  void setGains(double kp, double ki, double kd)
  {
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
  }

  double computeCommand(double error, double dt)
  {
    double currentE = error; // Error is already calculated in the function calls
    double deriv = (currentE - this->pastE) / dt;
    rsum += currentE * dt;
    double o = kp * currentE + kd * deriv + rsum * ki;
    this->pastE = currentE;
    return o;
  }

private:
  double kp = 0;
  double kd = 0;
  double ki = 0;
  double pastE = 0;
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

  void setUsingExternalVelocityTarget(bool using_external_target_vel) {
    this->using_external_target_vel = using_external_target_vel;
  }

  void setUsingAngles(bool using_angles) {
    this->using_angles = using_angles;
  }

  double computeCommand(double current_pos, double target_pos, double current_vel, double target_vel, double dt)
  {
    if (this->using_external_target_vel) {
      if (abs(target_vel) < 0.1) {
        target_pos = this->position_setpoint; // Hold at current position
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

private:
  PID position_controller;
  PID velocity_controller;

  double position_setpoint;
  bool using_external_target_vel = false; // Indicates if target velocity given vs computed from position
  bool using_angles = false; // Determines if positions are angle-wrapped

};

#endif
