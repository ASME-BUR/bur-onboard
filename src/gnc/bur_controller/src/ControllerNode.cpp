#include "ControllerNode.hpp"
#include <iostream>
namespace controller
{

  ControllerNode::ControllerNode() : rclcpp::Node::Node("bur_controller")
  {

    this->declare_parameter("sub_topic", "command");
    this->declare_parameter("pub_topic", "control_effort");
    this->declare_parameter("publish_rate", 100);
    this->declare_parameter("using_joy", true);
    this->declare_parameter("debug", false);

    this->declare_parameter("linear_x/p", 0.0);
    this->declare_parameter("linear_x/i", 0.0);
    this->declare_parameter("linear_x/d", 0.0);

    this->declare_parameter("linear_y/p", 0.0);
    this->declare_parameter("linear_y/i", 0.0);
    this->declare_parameter("linear_y/d", 0.0);

    this->declare_parameter("linear_z/p", 0.0);
    this->declare_parameter("linear_z/i", 0.0);
    this->declare_parameter("linear_z/d", 0.0);

    this->declare_parameter("angular_x/p", 0.0);
    this->declare_parameter("angular_x/i", 0.0);
    this->declare_parameter("angular_x/d", 0.0);

    this->declare_parameter("angular_y/p", 0.0);
    this->declare_parameter("angular_y/i", 0.0);
    this->declare_parameter("angular_y/d", 0.0);

    this->declare_parameter("angular_z/p", 0.0);
    this->declare_parameter("angular_z/i", 0.0);
    this->declare_parameter("angular_z/d", 0.0);

    state_setpoint_sub = this->create_subscription<bur_msgs::msg::Command>(
        this->get_parameter("sub_topic").as_string(), 1,
        std::bind(&ControllerNode::currentCommandCallback, this, std::placeholders::_1));

    param_callback = this->add_on_set_parameters_callback(bind(&ControllerNode::parametersCallback, this, std::placeholders::_1));
    pubControlEffort = this->create_publisher<geometry_msgs::msg::WrenchStamped>(this->get_parameter("pub_topic").as_string(), 1);

    int publish_rate = this->get_parameter("publish_rate").as_int();

    pubTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / publish_rate), std::bind(&ControllerNode::publishState, this));

    linear_x = CascadedPositionVelocityController();
    linear_y = CascadedPositionVelocityController();
    linear_z = CascadedPositionVelocityController();

    angular_x = CascadedPositionVelocityController();
    angular_y = CascadedPositionVelocityController();
    angular_z = CascadedPositionVelocityController();

    linear_x.setUsingExternalVelocityTarget(true);
    linear_y.setUsingExternalVelocityTarget(true);
    linear_z.setUsingExternalVelocityTarget(true);

    angular_x.setUsingExternalVelocityTarget(true);
    angular_y.setUsingExternalVelocityTarget(true);
    angular_z.setUsingExternalVelocityTarget(true);

    angular_x.setUsingAngles(true);
    angular_y.setUsingAngles(true);
    angular_z.setUsingAngles(true);

    set_constants();
  }

  void ControllerNode::set_constants()
  {
    using_joy = this->get_parameter("using_joy").as_bool();

    linear_x.setPositionGains(1.0, 0.0, 0.0);
    linear_y.setPositionGains(1.0, 0.0, 0.0);
    linear_z.setPositionGains(1.0, 0.0, 0.0);

    angular_x.setPositionGains(1.0, 0.0, 0.0);
    angular_y.setPositionGains(1.0, 0.0, 0.0);
    angular_z.setPositionGains(1.0, 0.0, 0.0);

    linear_x.setVelocityGains(this->get_parameter("linear_x/p").as_double(), this->get_parameter("linear_x/i").as_double(), this->get_parameter("linear_x/d").as_double());
    linear_y.setVelocityGains(this->get_parameter("linear_y/p").as_double(), this->get_parameter("linear_y/i").as_double(), this->get_parameter("linear_y/d").as_double());
    linear_z.setVelocityGains(this->get_parameter("linear_z/p").as_double(), this->get_parameter("linear_z/i").as_double(), this->get_parameter("linear_z/d").as_double());

    angular_x.setVelocityGains(this->get_parameter("angular_x/p").as_double(), this->get_parameter("angular_x/i").as_double(), this->get_parameter("angular_x/d").as_double());
    angular_y.setVelocityGains(this->get_parameter("angular_y/p").as_double(), this->get_parameter("angular_y/i").as_double(), this->get_parameter("angular_y/d").as_double());
    angular_z.setVelocityGains(this->get_parameter("angular_z/p").as_double(), this->get_parameter("angular_z/i").as_double(), this->get_parameter("angular_z/d").as_double());
  }

  rcl_interfaces::msg::SetParametersResult ControllerNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO(this->get_logger(), "%s: %s", param.get_name().c_str(), param.value_to_string().c_str());
    }
    set_constants();
    return result;
  }

  void ControllerNode::currentCommandCallback(const bur_msgs::msg::Command::SharedPtr msg)
  {
    pose_state = msg->current_pos.pose;
    pose_setpoint = msg->target_pos.pose;

    twist_state = msg->current_vel.twist;
    twist_setpoint = msg->target_vel.twist;

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

    active = (this->using_joy) ? (!msg->buttons.empty() && msg->buttons[9]): false;
  }

  void ControllerNode::publishState()
  {
    geometry_msgs::msg::WrenchStamped controlEffort; // All zeroes
    controlEffort.header.stamp = this->now();
    controlEffort.header.frame_id = "base_link";
    
    if (active) {
      rclcpp::Time time = this->now(); // might have to change the now ????
      double dt = (time - lastTime).seconds();

      if (lastTime.seconds() == 0 || dt == 0) {
        lastTime = time;
        return;
      }

      controlEffort.wrench.force.x = -linear_x.computeCommand(pose_state.position.x, pose_setpoint.position.x, twist_state.linear.x, twist_setpoint.linear.x, dt);
      controlEffort.wrench.force.y = -linear_y.computeCommand(pose_state.position.y, pose_setpoint.position.y, twist_state.linear.y, twist_setpoint.linear.y, dt);
      controlEffort.wrench.force.z = linear_z.computeCommand(pose_state.position.z, pose_setpoint.position.z, twist_state.linear.z, twist_setpoint.linear.z, dt);

      controlEffort.wrench.torque.x = angular_x.computeCommand(state_angle.getX(), setpoint_angle.getX(), twist_state.angular.x, twist_setpoint.angular.x, dt);
      controlEffort.wrench.torque.y = angular_y.computeCommand(state_angle.getY(), setpoint_angle.getY(), twist_state.angular.y, twist_setpoint.angular.y, dt);
      controlEffort.wrench.torque.z = angular_z.computeCommand(state_angle.getZ(), setpoint_angle.getZ(), twist_state.angular.z, twist_setpoint.angular.z, dt);

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
    pubControlEffort->publish(controlEffort);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<controller::ControllerNode>());

  rclcpp::shutdown();
  return 0;
}