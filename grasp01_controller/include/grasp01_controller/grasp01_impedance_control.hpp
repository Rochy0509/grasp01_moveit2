#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_lifecycle/state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace grasp01_impedance_control {

class JointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using VectorXd = Eigen::VectorXd;

  // Constructor
  JointImpedanceController();

  // Interface configurations for command and state interfaces
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Main update loop and lifecycle callbacks
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // Robot parameters
  std::string arm_id_;
  std::string robot_description_;
  int num_joints;  // Dynamically set based on configuration
  std::vector<std::string> joint_names_;

  // Joint state vectors
  VectorXd q_;               // Current joint positions
  VectorXd initial_q_;       // Home joint positions
  VectorXd dq_;              // Current joint velocities
  VectorXd dq_filtered_;     // Filtered velocities
  VectorXd desired_q_;       // Desired joint positions from IK

  // Control gains for impedance (PD) control
  VectorXd k_gains_;
  VectorXd d_gains_;

  // Cartesian pose for the end-effector
  Eigen::Vector3d initial_position_;     // Initial EE position (computed from home pose)
  Eigen::Quaterniond initial_orientation_; // Initial EE orientation (computed from home pose)
  Eigen::Vector3d desired_position_;     // Current desired EE position (from RViz or home)
  Eigen::Quaterniond desired_orientation_; // Current desired EE orientation (from RViz or home)

  // ROS components
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_; // For RViz poses

  // Flags and time
  bool initialization_flag_{true};
  double elapsed_time_{0.0};

  // Helper functions
  void updateJointStates();
  std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_ik_service_request(
      const Eigen::Vector3d& new_position,
      const Eigen::Quaterniond& new_orientation,
      const VectorXd& t_joint_positions);
  VectorXd compute_torque_command(const VectorXd& desired_joint_positions,
                                  const VectorXd& t_joint_positions);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg); // Handle RViz poses
};

}  // namespace grasp01_impedance_control