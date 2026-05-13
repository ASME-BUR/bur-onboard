#include <iostream>

#include "controller.hpp"

double CascadedPositionVelocityController::computeCommand(double current_pos, double target_pos, double current_vel, double target_vel, double dt)
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
