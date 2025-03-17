#include "grasp01_controller/grasp01_impedance_control.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

namespace grasp01_impedance_control {

JointImpedanceController::JointImpedanceController() = default;

// Command interface configuration: torque commands for each joint
controller_interface::InterfaceConfiguration JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

// State interface configuration: joint positions and velocities
controller_interface::InterfaceConfiguration JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

// Update joint states from state interfaces
void JointImpedanceController::updateJointStates() {
  for (int i = 0; i < num_joints; ++i) {
    q_(i) = state_interfaces_[i].get_value();              // Joint positions
    dq_(i) = state_interfaces_[i + num_joints].get_value(); // Joint velocities
  }
}

// Create IK service request
std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> JointImpedanceController::create_ik_service_request(
    const Eigen::Vector3d& new_position,
    const Eigen::Quaterniond& new_orientation,
    const VectorXd& t_joint_positions) {
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  request->ik_request.group_name = arm_id_;
  request->ik_request.pose_stamped.header.frame_id = "base_link";
  request->ik_request.pose_stamped.pose.position.x = new_position.x();
  request->ik_request.pose_stamped.pose.position.y = new_position.y();
  request->ik_request.pose_stamped.pose.position.z = new_position.z();
  request->ik_request.pose_stamped.pose.orientation.x = new_orientation.x();
  request->ik_request.pose_stamped.pose.orientation.y = new_orientation.y();
  request->ik_request.pose_stamped.pose.orientation.z = new_orientation.z();
  request->ik_request.pose_stamped.pose.orientation.w = new_orientation.w();
  request->ik_request.robot_state.joint_state.name = joint_names_;
  request->ik_request.robot_state.joint_state.position.assign(t_joint_positions.data(),
                                                             t_joint_positions.data() + num_joints);
  return request;
}

// Compute torque based on impedance control law
JointImpedanceController::VectorXd JointImpedanceController::compute_torque_command(
    const VectorXd& desired_joint_positions,
    const VectorXd& current_joint_positions) {
  VectorXd error = desired_joint_positions - current_joint_positions;
  return k_gains_.cwiseProduct(error) - d_gains_.cwiseProduct(dq_);
}

// Callback for receiving pose commands from RViz
void JointImpedanceController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  desired_position_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  desired_orientation_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                            msg->pose.orientation.y, msg->pose.orientation.z);

  auto ik_request = create_ik_service_request(desired_position_, desired_orientation_, q_);
  auto future = compute_ik_client_->async_send_request(ik_request);
  future.wait();  // Blocking wait for simplicity

  auto response = future.get();
  if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    for (int i = 0; i < num_joints; ++i) {
      desired_q_(i) = response->solution.joint_state.position[i];
    }
    RCLCPP_INFO(get_node()->get_logger(), "Updated desired joint positions from IK");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "IK failed with error code: %d", response->error_code.val);
  }
}

// Main update loop: compute and apply torques
controller_interface::return_type JointImpedanceController::update(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/) {
  updateJointStates();
  VectorXd tau = compute_torque_command(desired_q_, q_);
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau(i));
  }
  return controller_interface::return_type::OK;
}

// Lifecycle callback: on_init
CallbackReturn JointImpedanceController::on_init() {
  return CallbackReturn::SUCCESS;
}

// Lifecycle callback: on_configure
CallbackReturn JointImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  try {
    // Retrieve joint names dynamically from the YAML file
    joint_names_ = node->get_parameter("joints").as_string_array();
    num_joints = joint_names_.size();

    // Retrieve impedance gains dynamically from the YAML file
    auto k_gains_param = node->get_parameter("k_gains").as_double_array();
    auto d_gains_param = node->get_parameter("d_gains").as_double_array();

    // Validate gain sizes match the number of joints
    if (k_gains_param.size() != static_cast<size_t>(num_joints)) {
      RCLCPP_ERROR(node->get_logger(), "k_gains must have %d elements, got %zu", num_joints, k_gains_param.size());
      return CallbackReturn::FAILURE;
    }
    if (d_gains_param.size() != static_cast<size_t>(num_joints)) {
      RCLCPP_ERROR(node->get_logger(), "d_gains must have %d elements, got %zu", num_joints, d_gains_param.size());
      return CallbackReturn::FAILURE;
    }

    // Assign gains to Eigen vectors
    k_gains_ = Eigen::Map<Eigen::VectorXd>(k_gains_param.data(), num_joints);
    d_gains_ = Eigen::Map<Eigen::VectorXd>(d_gains_param.data(), num_joints);

    // Initialize state vectors based on the number of joints
    q_ = VectorXd::Zero(num_joints);
    dq_ = VectorXd::Zero(num_joints);
    desired_q_ = VectorXd::Zero(num_joints);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  // Set up ROS clients and subscribers
  compute_ik_client_ = node->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  pose_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_group/goal", 10,
      std::bind(&JointImpedanceController::pose_callback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

// Lifecycle callback: on_activate
CallbackReturn JointImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!compute_ik_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "IK service not available");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

// Lifecycle callback: on_deactivate
CallbackReturn JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // Reset torque commands to zero
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace grasp01_impedance_control

// Register the controller as a ROS 2 plugin
PLUGINLIB_EXPORT_CLASS(grasp01_impedance_control::JointImpedanceController, controller_interface::ControllerInterface)