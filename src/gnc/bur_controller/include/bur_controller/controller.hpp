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

  void reset()
  {
    this->pastE = 0.0;
    this->rsum = 0.0;
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

  double computeCommand(double current_pos, double target_pos, double current_vel, double target_vel, double dt);

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

  bool using_external_target_vel = false; // Target velocity given vs. computed from position
  bool using_angles = false; // Determines if positions are angle-wrapped

};

#endif
