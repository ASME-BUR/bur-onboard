#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "controller_node.hpp"

ControllerNode::ControllerNode() : rclcpp::Node::Node("bur_controller")
{

  this->declare_parameter("sub_topic", "command");
  this->declare_parameter("pub_topic", "control_effort");
  this->declare_parameter("publish_rate", 100);
  this->declare_parameter("using_joy", true);
  this->declare_parameter("debug", false);

  // Declare PID coefficients for our 6 degrees of freedom (linear XYZ + roll, pitch, yaw) in a loop
  // Each axis has 2 controllers (one position PID + one velocity PID) -> 6 coefficients
  std::vector<std::string> dofs = {"linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"};
  for (int i = 0; i < dofs.size(); i++) {
    this->declare_parameter(dofs[i] + "/position/kp", 0.0);
    this->declare_parameter(dofs[i] + "/position/ki", 0.0);
    this->declare_parameter(dofs[i] + "/position/kd", 0.0);
    this->declare_parameter(dofs[i] + "/velocity/kp", 0.0);
    this->declare_parameter(dofs[i] + "/velocity/ki", 0.0);
    this->declare_parameter(dofs[i] + "/velocity/kd", 0.0);
  }

  this->state_setpoint_sub = this->create_subscription<bur_msgs::msg::Command>(
      this->get_parameter("sub_topic").as_string(), 1,
      std::bind(&ControllerNode::currentCommandCallback, this, std::placeholders::_1));

  this->param_callback = this->add_on_set_parameters_callback(bind(&ControllerNode::parametersCallback, this, std::placeholders::_1));
  this->pubControlEffort = this->create_publisher<geometry_msgs::msg::WrenchStamped>(this->get_parameter("pub_topic").as_string(), 1);

  int publish_rate = this->get_parameter("publish_rate").as_int();

  this->pubTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate), std::bind(&ControllerNode::publishState, this));

  this->linear_x = CascadedPositionVelocityController();
  this->linear_y = CascadedPositionVelocityController();
  this->linear_z = CascadedPositionVelocityController();

  this->angular_x = CascadedPositionVelocityController();
  this->angular_y = CascadedPositionVelocityController();
  this->angular_z = CascadedPositionVelocityController();

  angular_x.setUsingAngles(true);
  angular_y.setUsingAngles(true);
  angular_z.setUsingAngles(true);

  set_constants();
}


/**
 * Sets PID coefficients from ROS parameters
 */
void ControllerNode::set_constants()
{
  // Map controllers to their ROS parameter names for convenience
  std::vector<std::pair<std::string, CascadedPositionVelocityController*>> controllers = {
    std::make_pair<std::string, CascadedPositionVelocityController*>("linear_x", &this->linear_x),
    std::make_pair<std::string, CascadedPositionVelocityController*>("linear_y", &this->linear_y),
    std::make_pair<std::string, CascadedPositionVelocityController*>("linear_z", &this->linear_z),
    std::make_pair<std::string, CascadedPositionVelocityController*>("angular_x", &this->angular_x),
    std::make_pair<std::string, CascadedPositionVelocityController*>("angular_y", &this->angular_y),
    std::make_pair<std::string, CascadedPositionVelocityController*>("angular_z", &this->angular_z),
  };

  // Set PID coefficients for our controllers from ROS parameters in a loop
  this->using_joy = this->get_parameter("using_joy").as_bool();
  for (int i = 0; i < controllers.size(); i++) {
    controllers[i].second->setPositionGains(
      this->get_parameter(controllers[i].first + "/position/kp").as_double(),
      this->get_parameter(controllers[i].first + "/position/ki").as_double(),
      this->get_parameter(controllers[i].first + "/position/kd").as_double()
    );
    controllers[i].second->setVelocityGains(
      this->get_parameter(controllers[i].first + "/velocity/kp").as_double(),
      this->get_parameter(controllers[i].first + "/velocity/ki").as_double(),
      this->get_parameter(controllers[i].first + "/velocity/kd").as_double()
    );

    // Determines whether target velocity is given or computed by the controller itself
    // Corresponds to tele-op (target velocity given) vs autonomous (no target given)
    controllers[i].second->setUsingExternalVelocityTarget(this->using_joy);
  }
}


/**
 * Updates PID coefficients when ROS node parameters are updated (e.g. via rqt)
 */
rcl_interfaces::msg::SetParametersResult ControllerNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : parameters)
  {
    RCLCPP_INFO(this->get_logger(), "%s: %s", param.get_name().c_str(), param.value_to_string().c_str());
  }
  this->set_constants();
  return result;
}


/**
 * Compute current + target position, orientation, velocities, angular velocities
 */
void ControllerNode::currentCommandCallback(const bur_msgs::msg::Command::SharedPtr msg)
{
  // Compute current + target position, orientation, velocities, angular velocities
  this->pose_state = msg->current_pos.pose;
  this->pose_setpoint = msg->target_pos.pose;

  this->twist_state = msg->current_vel.twist;
  this->twist_setpoint = msg->target_vel.twist;

  // Compute current orientation (quat -> RPY)
  double roll_state, pitch_state, yaw_state;
  tf2::Quaternion q = tf2::Quaternion(msg->current_pos.pose.orientation.x, msg->current_pos.pose.orientation.y, msg->current_pos.pose.orientation.z, msg->current_pos.pose.orientation.w);
  tf2::Matrix3x3 rot_matrix = tf2::Matrix3x3(q);
  rot_matrix.getRPY(roll_state, pitch_state, yaw_state);
  this->state_angle = tf2::Vector3(roll_state, pitch_state, yaw_state);

  // Compute target orientation (quat -> RPY)
  double roll_setpoint, pitch_setpoint, yaw_setpoint;
  q = tf2::Quaternion(msg->target_pos.pose.orientation.x, msg->target_pos.pose.orientation.y, msg->target_pos.pose.orientation.z, msg->target_pos.pose.orientation.w);
  rot_matrix = tf2::Matrix3x3(q);
  rot_matrix.getRPY(roll_setpoint, pitch_setpoint, yaw_setpoint);
  this->setpoint_angle = tf2::Vector3(roll_setpoint, pitch_setpoint, yaw_setpoint);

  this->active = (this->using_joy) ? (!msg->buttons.empty() && msg->buttons[9]): false;
}


/**
 * Publish control effort from current + target position, orientation, etc.
 */
void ControllerNode::publishState()
{
  geometry_msgs::msg::WrenchStamped controlEffort;
  controlEffort.header.stamp = this->now();
  controlEffort.header.frame_id = "base_link";

  if (active) {
    rclcpp::Time time = this->now(); // might have to change the now ????
    if (lastTime.seconds() == 0) {
      lastTime = time;
      return;
    }
    double dt = (time - lastTime).seconds();

    // Compute translational control effort
    controlEffort.wrench.force.x = -linear_x.computeCommand(pose_state.position.x, pose_setpoint.position.x, twist_state.linear.x, twist_setpoint.linear.x, dt);
    controlEffort.wrench.force.y = -linear_y.computeCommand(pose_state.position.y, pose_setpoint.position.y, twist_state.linear.y, twist_setpoint.linear.y, dt);
    controlEffort.wrench.force.z = linear_z.computeCommand(pose_state.position.z, pose_setpoint.position.z, twist_state.linear.z, twist_setpoint.linear.z, dt);

    // Compute rotational control effort
    controlEffort.wrench.torque.x = angular_x.computeCommand(state_angle.getX(), setpoint_angle.getX(), twist_state.angular.x, twist_setpoint.angular.x, dt);
    controlEffort.wrench.torque.y = angular_y.computeCommand(state_angle.getY(), setpoint_angle.getY(), twist_state.angular.y, twist_setpoint.angular.y, dt);
    controlEffort.wrench.torque.z = angular_z.computeCommand(state_angle.getZ(), setpoint_angle.getZ(), twist_state.angular.z, twist_setpoint.angular.z, dt);

    // Print statements for debugging
    if(this->get_parameter("debug").as_bool()) {
      std::cout << "\033[2J\033[1;1H" << std::endl;
      std::cout << "linear x velocity: " << twist_state.linear.x << " EXPECTED: " << twist_setpoint.linear.x << std::endl;
      std::cout << "linear y velocity: " << twist_state.linear.y << " EXPECTED: " << twist_setpoint.linear.y << std::endl; 
      std::cout << "linear z position: " << pose_state.position.z << " EXPECTED: " << pose_setpoint.position.z << std::endl;
      std::cout << "yaw (z) angle (pos): " << state_angle.getZ() << " EXPECTED (if yaw_hold = true): " << setpoint_angle.getZ() << std::endl;
      std::cout << "yaw (z) twist (vel): " << twist_state.angular.z << " EXPECTED (if yaw_hold = false): " << twist_setpoint.angular.z << std::endl;
      std::cout << "roll (x) angle (pos): " << state_angle.getX() << " EXPECTED (if roll_hold = true): " << setpoint_angle.getX() << std::endl;
      std::cout << "roll (x) twist (vel): " << twist_state.angular.x << " EXPECTED (if roll_hold = false): " << twist_setpoint.angular.x << std::endl;
      std::cout << "pitch (y) angle (pos): " << state_angle.getY() << " EXPECTED (if pitch_hold = true): " << setpoint_angle.getY() << std::endl;
      std::cout << "pitch (y) twist (vel): " << twist_state.angular.y << " EXPECTED (if pitch_hold = false): " << twist_setpoint.angular.y << std::endl;
    }
  }
  pubControlEffort->publish(controlEffort); // Not active -> all zeroes
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
