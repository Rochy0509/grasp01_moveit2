#include "grasp01_controller/grasp01_impedance_control.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <functional>
#include <sstream>
#include <pinocchio/algorithm/rnea.hpp>

namespace grasp01_impedance_control {

// Helper function to convert an Eigen vector to a string
template<typename Derived>
std::string eigen_to_string(const Eigen::MatrixBase<Derived>& vec) {
  std::stringstream ss;
  ss << vec.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "[", "]", "[", "]"));
  return ss.str();
}

JointImpedanceController::JointImpedanceController()
    : num_joints_(0),
      initialization_flag_(true),
      elapsed_time_(0.0) {
  q_ = Eigen::VectorXd::Zero(6);
  initial_q_ = Eigen::VectorXd::Zero(6);
  dq_ = Eigen::VectorXd::Zero(6);
  dq_filtered_ = Eigen::VectorXd::Zero(6);
  desired_q_ = Eigen::VectorXd::Zero(6);
  k_gains_ = Eigen::VectorXd::Zero(6);
  d_gains_ = Eigen::VectorXd::Zero(6);
}

controller_interface::InterfaceConfiguration JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

void JointImpedanceController::updateJointStates() {
  if (num_joints_ == 0) return;
  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    q_(i) = state_interfaces_[i].get_value();
    dq_(i) = state_interfaces_[i + num_joints_].get_value();
  }
  const double alpha = 0.2;
  if (initialization_flag_) {
    dq_filtered_ = dq_;
    initialization_flag_ = false;
  } else {
    dq_filtered_ = alpha * dq_ + (1.0 - alpha) * dq_filtered_;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Updated joint states: q_ = %s, dq_ = %s",
               eigen_to_string(q_).c_str(),
               eigen_to_string(dq_).c_str());
}

std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> JointImpedanceController::create_ik_service_request(
    const geometry_msgs::msg::Pose& pose,
    const std::vector<std::string>& joint_names,
    const Eigen::VectorXd& current_joint_positions) {
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  request->ik_request.group_name = arm_id_;
  request->ik_request.pose_stamped.header.frame_id = "base_link";
  request->ik_request.pose_stamped.pose = pose;
  request->ik_request.robot_state.joint_state.name = joint_names;
  request->ik_request.robot_state.joint_state.position.assign(
      current_joint_positions.data(), current_joint_positions.data() + num_joints_);
  return request;
}

Eigen::VectorXd JointImpedanceController::compute_torque_command(
    const Eigen::VectorXd& desired_joint_positions,
    const Eigen::VectorXd& current_joint_positions,
    const Eigen::VectorXd& current_velocities) {
  Eigen::VectorXd error = desired_joint_positions - current_joint_positions;
  Eigen::VectorXd gravity_torques = get_gravity_torques(current_joint_positions);
  Eigen::VectorXd tau = k_gains_.cwiseProduct(error) - d_gains_.cwiseProduct(current_velocities) + gravity_torques;
  RCLCPP_DEBUG(get_node()->get_logger(), "Computed torques: stiffness=%s, damping=%s, gravity=%s, total=%s",
               eigen_to_string(k_gains_.cwiseProduct(error)).c_str(),
               eigen_to_string(-d_gains_.cwiseProduct(current_velocities)).c_str(),
               eigen_to_string(gravity_torques).c_str(),
               eigen_to_string(tau).c_str());
  return tau;
}

Eigen::VectorXd JointImpedanceController::get_gravity_torques(const Eigen::VectorXd& joint_positions) {
  pinocchio::Data::VectorXs q = joint_positions.cast<pinocchio::Data::Scalar>();
  // Use rnea with zero velocity and acceleration to compute gravity torques
  pinocchio::Data::VectorXs tau_g = pinocchio::rnea(model_, *data_, q,
                                                    pinocchio::Data::VectorXs::Zero(model_.nv),
                                                    pinocchio::Data::VectorXs::Zero(model_.nv));
  return tau_g.cast<double>();
}

rclcpp_action::GoalResponse JointImpedanceController::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory goal with %zu points", goal->trajectory.points.size());
  if (goal->trajectory.joint_names.empty() || goal->trajectory.points.empty() || goal->trajectory.joint_names != joint_names_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid trajectory: empty joint names, no points, or joint names mismatch");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointImpedanceController::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(get_node()->get_logger(), "Canceling trajectory");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointImpedanceController::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
  std::thread{std::bind(&JointImpedanceController::execute_trajectory, this, goal_handle), std::move(goal_handle)}.detach();
}

void JointImpedanceController::execute_trajectory(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  rclcpp::Rate loop_rate(100);
  for (const auto& point : goal->trajectory.points) {
    Eigen::VectorXd desired_positions = Eigen::Map<const Eigen::VectorXd>(point.positions.data(), num_joints_);
    geometry_msgs::msg::Pose pose;
    auto ik_request = create_ik_service_request(pose, joint_names_, desired_positions);
    auto future = compute_ik_client_->async_send_request(ik_request);

    if (rclcpp::spin_until_future_complete(get_node(), future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        desired_q_ = Eigen::Map<const Eigen::VectorXd>(response->solution.joint_state.position.data(), num_joints_);
        RCLCPP_INFO(get_node()->get_logger(), "Updated desired joint positions from IK: %s",
                    eigen_to_string(desired_q_).c_str());
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "IK failed with error code: %d", response->error_code.val);
        goal_handle->abort(result);
        return;
      }
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "IK service call failed");
      goal_handle->abort(result);
      return;
    }

    while (rclcpp::ok() && !goal_handle->is_canceling()) {
      updateJointStates();
      Eigen::VectorXd tau = compute_torque_command(desired_q_, q_, dq_filtered_);
      for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
        command_interfaces_[i].set_value(tau(i));
      }
      feedback->actual.positions.assign(q_.data(), q_.data() + num_joints_);
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }
  }

  result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
  goal_handle->succeed(result);
}

controller_interface::return_type JointImpedanceController::update(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period) {
  (void)time;
  elapsed_time_ += period.seconds();
  updateJointStates();

  if (initialization_flag_ && elapsed_time_ > 1.0) {
    initial_q_ = q_;
    desired_q_ = initial_q_;
    initialization_flag_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Initialized desired_q_ to current pose: %s",
                eigen_to_string(desired_q_).c_str());
  }

  Eigen::VectorXd tau = compute_torque_command(desired_q_, q_, dq_filtered_);
  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    command_interfaces_[i].set_value(tau(i));
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Applied torque: %s",
               eigen_to_string(tau).c_str());

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn JointImpedanceController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  try {
    arm_id_ = node->get_parameter("arm_id").as_string();
    robot_description_ = node->get_parameter("robot_description").as_string();
    joint_names_ = node->get_parameter("joints").as_string_array();
    num_joints_ = joint_names_.size();

    auto k_gains_param = node->get_parameter("k_gains").as_double_array();
    auto d_gains_param = node->get_parameter("d_gains").as_double_array();

    if (k_gains_param.size() != static_cast<size_t>(num_joints_)) {
      RCLCPP_ERROR(node->get_logger(), "k_gains must have %zu elements, got %zu",
                   static_cast<size_t>(num_joints_), k_gains_param.size());
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (d_gains_param.size() != static_cast<size_t>(num_joints_)) {
      RCLCPP_ERROR(node->get_logger(), "d_gains must have %zu elements, got %zu",
                   static_cast<size_t>(num_joints_), d_gains_param.size());
      return controller_interface::CallbackReturn::FAILURE;
    }

    k_gains_ = Eigen::Map<const Eigen::VectorXd>(k_gains_param.data(), num_joints_);
    d_gains_ = Eigen::Map<const Eigen::VectorXd>(d_gains_param.data(), num_joints_);

    q_ = Eigen::VectorXd::Zero(num_joints_);
    initial_q_ = Eigen::VectorXd::Zero(num_joints_);
    dq_ = Eigen::VectorXd::Zero(num_joints_);
    dq_filtered_ = Eigen::VectorXd::Zero(num_joints_);
    desired_q_ = Eigen::VectorXd::Zero(num_joints_);

    std::string urdf_string;
    if (!node->get_parameter(robot_description_, urdf_string)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get robot_description parameter");
      return controller_interface::CallbackReturn::FAILURE;
    }
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);
    RCLCPP_INFO(node->get_logger(), "Loaded Pinocchio model with %d joints", model_.nq);

    action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        get_node(),
        "follow_joint_trajectory",
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
          return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
          return this->handle_cancel(goal_handle);
        },
        [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
          this->handle_accepted(goal_handle);
        });

    compute_ik_client_ = node->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure: %s", e.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!compute_ik_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "IK service not available");
    return controller_interface::CallbackReturn::FAILURE;
  }

  updateJointStates();
  initial_q_ = q_;
  desired_q_ = initial_q_;
  RCLCPP_INFO(get_node()->get_logger(), "Initialized joint states: q_ = %s, desired_q_ = %s",
              eigen_to_string(q_).c_str(),
              eigen_to_string(desired_q_).c_str());

  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "Set initial torques to zero");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointImpedanceController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  desired_position_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  desired_orientation_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                           msg->pose.orientation.y, msg->pose.orientation.z);

  auto ik_request = create_ik_service_request(msg->pose, joint_names_, q_);
  auto future = compute_ik_client_->async_send_request(ik_request);
  if (rclcpp::spin_until_future_complete(get_node(), future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      desired_q_ = Eigen::Map<const Eigen::VectorXd>(response->solution.joint_state.position.data(), num_joints_);
      RCLCPP_INFO(get_node()->get_logger(), "Updated desired joint positions from RViz pose: %s",
                  eigen_to_string(desired_q_).c_str());
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "IK failed with error code: %d", response->error_code.val);
    }
  }
}

}  // namespace grasp01_impedance_control

PLUGINLIB_EXPORT_CLASS(grasp01_impedance_control::JointImpedanceController, controller_interface::ControllerInterface)