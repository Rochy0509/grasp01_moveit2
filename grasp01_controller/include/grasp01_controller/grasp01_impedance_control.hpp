#ifndef GRASP01_IMPEDANCE_CONTROL_HPP_
#define GRASP01_IMPEDANCE_CONTROL_HPP_

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace grasp01_impedance_control {

class JointImpedanceController : public controller_interface::ControllerInterface {
public:
  using VectorXd = Eigen::VectorXd;  // Ensure the type alias is defined
  JointImpedanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

protected:
  void updateJointStates();
  std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_ik_service_request(
      const geometry_msgs::msg::Pose& pose,
      const std::vector<std::string>& joint_names,
      const Eigen::VectorXd& current_joint_positions);
  Eigen::VectorXd compute_torque_command(const Eigen::VectorXd& desired_joint_positions,
                                         const Eigen::VectorXd& current_joint_positions,
                                         const Eigen::VectorXd& current_velocities);
  Eigen::VectorXd get_gravity_torques(const Eigen::VectorXd& joint_positions);

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
  void execute_trajectory(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

  int num_joints_;
  std::vector<std::string> joint_names_;
  std::string arm_id_;
  std::string robot_description_;
  bool initialization_flag_;
  double elapsed_time_;
  Eigen::VectorXd q_;              // Current joint positions
  Eigen::VectorXd initial_q_;      // Home joint positions
  Eigen::VectorXd dq_;             // Current joint velocities
  Eigen::VectorXd dq_filtered_;    // Filtered joint velocities
  Eigen::VectorXd desired_q_;      // Desired joint positions
  Eigen::VectorXd k_gains_;        // Proportional gains
  Eigen::VectorXd d_gains_;        // Derivative gains
  Eigen::Vector3d desired_position_;
  Eigen::Quaterniond desired_orientation_;

  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
};

}  // namespace grasp01_impedance_control

#endif  // GRASP01_IMPEDANCE_CONTROL_HPP_