#include "grasp01_hardware/grasp01_hardware_interface.hpp"
#include <cmath>
#include <fstream>
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace grasp01_hardware {

rclcpp::Logger Grasp01HardwareInterface::getLogger() {
    return rclcpp::get_logger("Grasp01HardwareInterface");
}

Grasp01HardwareInterface::~Grasp01HardwareInterface() {
    stopAsyncThread();
    if (driver_) {
        for (auto& actuator : actuators_) {
            if (actuator) {
                actuator->shutdownMotor();
            }
        }
    }
}

hardware_interface::CallbackReturn Grasp01HardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info.joints.size() != 6) {
        RCLCPP_FATAL(getLogger(), "Expected 6 joints for grasp01 arm but got %zu.", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    ifname_ = info.hardware_parameters.count("ifname") ? info.hardware_parameters.at("ifname") : "";
    if (ifname_.empty()) {
        RCLCPP_FATAL(getLogger(), "Could not parse CAN interface name!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    cycle_time_ = info.hardware_parameters.count("cycle_time") ?
        std::chrono::milliseconds(std::stol(info.hardware_parameters.at("cycle_time"))) :
        std::chrono::milliseconds(1);
    RCLCPP_INFO(getLogger(), "Cycle time set to %ld ms for real-time control.", cycle_time_.count());
    timeout_ = info.hardware_parameters.count("timeout") ?
        std::chrono::milliseconds(std::stoi(info.hardware_parameters.at("timeout"))) :
        std::chrono::milliseconds(0);

    if (cycle_time_.count() < 1) {
        RCLCPP_WARN(getLogger(), "Cycle time %ld ms may be too short for CAN bus reliability. Adjust if needed.", cycle_time_.count());
    }

    actuator_ids_.resize(6);
    torque_constants_.resize(6);
    max_velocities_.resize(6);
    joint_names_.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        joint_names_[i] = info.joints[i].name;
        auto const& params = info.joints[i].parameters;
        actuator_ids_[i] = params.count("actuator_id") ? std::stoi(params.at("actuator_id")) : 0;
        if (actuator_ids_[i] == 0) {
            RCLCPP_FATAL(getLogger(), "Missing 'actuator_id' for joint '%s'!", joint_names_[i].c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        torque_constants_[i] = params.count("torque_constant") ?
            std::stod(params.at("torque_constant")) : std::numeric_limits<double>::quiet_NaN();
        max_velocities_[i] = params.count("max_velocity") ? std::stod(params.at("max_velocity")) : 720.0;
    }

    position_states_.resize(6, 0.0);
    velocity_states_.resize(6, 0.0);
    effort_states_.resize(6, 0.0);
    position_commands_.resize(6, 0.0);
    velocity_commands_.resize(6, 0.0);
    effort_commands_.resize(6, 0.0);

    position_interface_running_ = false;
    velocity_interface_running_ = false;
    effort_interface_running_ = false;
    stop_async_thread_ = false;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Grasp01HardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    driver_ = std::make_unique<myactuator_rmd::CanDriver>(ifname_);
    actuators_.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        actuators_[i] = std::make_unique<myactuator_rmd::ActuatorInterface>(*driver_, actuator_ids_[i]);
    }
    if (!startAsyncThread(cycle_time_)) {
        RCLCPP_FATAL(getLogger(), "Failed to start async thread for real-time control!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Grasp01HardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    stopAsyncThread();
    for (auto& actuator : actuators_) {
        if (actuator) {
            actuator->shutdownMotor();
        }
    }
    driver_.reset();
    actuators_.clear();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Grasp01HardwareInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    stopAsyncThread();
    for (auto& actuator : actuators_) {
        if (actuator) {
            actuator->shutdownMotor();
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Grasp01HardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(getLogger(), "Activating grasp01 hardware interface for real-time control.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Grasp01HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    stopAsyncThread();
    for (auto& actuator : actuators_) {
        if (actuator) {
            actuator->stopMotor();
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Grasp01HardwareInterface::on_error(const rclcpp_lifecycle::State& /*previous_state*/) {
    stopAsyncThread();
    for (auto& actuator : actuators_) {
        if (actuator) {
            actuator->stopMotor();
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Grasp01HardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < 6; ++i) {
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_states_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Grasp01HardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < 6; ++i) {
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]);
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_commands_[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type Grasp01HardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces [[maybe_unused]]) {
    std::string requested_mode;
    for (const auto& iface : start_interfaces) {
        size_t pos = iface.find('/');
        if (pos != std::string::npos) {
            std::string mode = iface.substr(pos + 1);
            if (requested_mode.empty()) {
                requested_mode = mode;
            } else if (mode != requested_mode) {
                RCLCPP_ERROR(getLogger(), "Inconsistent modes detected in start_interfaces!");
                return hardware_interface::return_type::ERROR;
            }
        }
    }
    if (!start_interfaces.empty() && start_interfaces.size() != 6) {
        RCLCPP_ERROR(getLogger(), "Expected 6 interfaces for mode switch, got %zu.", start_interfaces.size());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Grasp01HardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces [[maybe_unused]]) {
    position_interface_running_ = false;
    velocity_interface_running_ = false;
    effort_interface_running_ = false;

    if (!start_interfaces.empty()) {
        std::string mode = start_interfaces[0].substr(start_interfaces[0].find('/') + 1);
        if (mode == hardware_interface::HW_IF_POSITION) {
            position_interface_running_ = true;
            RCLCPP_INFO(getLogger(), "Switched to position mode for all joints.");
        } else if (mode == hardware_interface::HW_IF_VELOCITY) {
            velocity_interface_running_ = true;
            RCLCPP_INFO(getLogger(), "Switched to velocity mode for all joints.");
        } else if (mode == hardware_interface::HW_IF_EFFORT) {
            effort_interface_running_ = true;
            RCLCPP_INFO(getLogger(), "Switched to effort mode for all joints.");
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Grasp01HardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    for (size_t i = 0; i < 6; ++i) {
        // Store previous values to detect changes
        double prev_position = position_states_[i];
        
        // Update current values
        position_states_[i] = async_position_states_[i].load();
        velocity_states_[i] = async_velocity_states_[i].load();
        effort_states_[i] = async_effort_states_[i].load();
        
        // Log state changes to help debug
        if (std::abs(prev_position - position_states_[i]) > 0.001) {
            RCLCPP_DEBUG(getLogger(), 
                "Joint %s position updated: %.6f -> %.6f", 
                joint_names_[i].c_str(),
                prev_position, 
                position_states_[i]);
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Grasp01HardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    if (position_interface_running_) {
        for (size_t i = 0; i < 6; ++i) {
            async_position_commands_[i].store(position_commands_[i]);
        }
    } else if (velocity_interface_running_) {
        for (size_t i = 0; i < 6; ++i) {
            async_velocity_commands_[i].store(velocity_commands_[i]);
        }
    } else if (effort_interface_running_) {
        for (size_t i = 0; i < 6; ++i) {
            async_effort_commands_[i].store(effort_commands_[i]);
        }
    }
    return hardware_interface::return_type::OK;
}

void Grasp01HardwareInterface::asyncThread(std::chrono::milliseconds const& cycle_time) {
    for (auto& actuator : actuators_) {
        actuator->setTimeout(timeout_);
    }
    
    // Track previous states to detect changes
    std::array<double, 6> prev_positions{};
    
    while (!stop_async_thread_) {
        auto const now = std::chrono::steady_clock::now();
        auto const wakeup_time = now + cycle_time;
        
        // Update motor control commands
        for (size_t i = 0; i < 6; ++i) {
            if (position_interface_running_) {
                feedback_[i] = actuators_[i]->sendPositionAbsoluteSetpoint(
                    grasp01_hardware::radToDeg(async_position_commands_[i].load()), max_velocities_[i]);
            } else if (velocity_interface_running_) {
                feedback_[i] = actuators_[i]->sendVelocitySetpoint(
                    grasp01_hardware::radToDeg(async_velocity_commands_[i].load()));
            } else if (effort_interface_running_) {
                feedback_[i] = actuators_[i]->sendTorqueSetpoint(
                    async_effort_commands_[i].load(), torque_constants_[i]);
            } else {
                // Even if no command is running, actively poll motor status
                feedback_[i] = actuators_[i]->getMotorStatus2();
            }

            // Store previous value to detect changes
            prev_positions[i] = async_position_states_[i].load();
            
            // Update the state with latest feedback
            async_position_states_[i].store(grasp01_hardware::degToRad(feedback_[i].shaft_angle));
            async_velocity_states_[i].store(grasp01_hardware::degToRad(feedback_[i].shaft_speed));
            async_effort_states_[i].store(grasp01_hardware::currentToTorque(feedback_[i].current, torque_constants_[i]));
            
            // Log significant changes in position
            if (std::abs(prev_positions[i] - async_position_states_[i].load()) > 0.01) {
                RCLCPP_DEBUG(getLogger(), 
                    "Motor %d position changed: %.6f -> %.6f", 
                    actuator_ids_[i],
                    prev_positions[i], 
                    async_position_states_[i].load());
            }
        }
        
        std::this_thread::sleep_until(wakeup_time);
    }
    
    for (auto& actuator : actuators_) {
        actuator->setTimeout(std::chrono::milliseconds(0));
    }
}

bool Grasp01HardwareInterface::startAsyncThread(std::chrono::milliseconds const& cycle_time) {
    try {
        async_thread_ = std::thread(&Grasp01HardwareInterface::asyncThread, this, cycle_time);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Failed to start async thread: %s", e.what());
        return false;
    }

#ifdef GRASP01_HARDWARE__THREAD_PRIORITY
    std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
    bool has_realtime = false;
    if (realtime_file.is_open()) {
        realtime_file >> has_realtime;
    }

    int policy{};
    struct ::sched_param param{};
    ::pthread_getschedparam(async_thread_.native_handle(), &policy, &param);
    if (has_realtime) {
        policy = SCHED_FIFO;
        RCLCPP_INFO(getLogger(), "Real-time system detected: Setting policy to 'SCHED_FIFO'...");
    }
    int const max_thread_priority = ::sched_get_priority_max(policy);
    if (max_thread_priority != -1) {
        param.sched_priority = max_thread_priority;
        if (::pthread_setschedparam(async_thread_.native_handle(), policy, &param) == 0) {
            RCLCPP_INFO(getLogger(), "Set thread priority '%d' and policy '%d' to async thread for real-time control!",
                param.sched_priority, policy);
        } else {
            RCLCPP_WARN(getLogger(), "Failed to set thread priority '%d' and policy '%d' to async thread!",
                param.sched_priority, policy);
        }
    } else {
        RCLCPP_WARN(getLogger(), "Could not set thread priority to async thread: Failed to get max priority!");
    }
#endif
    return true;
}

void Grasp01HardwareInterface::stopAsyncThread() {
    stop_async_thread_ = true;
    if (async_thread_.joinable()) {
        async_thread_.join();
    }
}

}  // namespace grasp01_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(grasp01_hardware::Grasp01HardwareInterface, hardware_interface::SystemInterface)
