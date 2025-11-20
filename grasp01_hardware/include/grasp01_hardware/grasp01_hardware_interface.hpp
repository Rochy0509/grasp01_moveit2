#ifndef GRASP01_HARDWARE_INTERFACE_HPP
#define GRASP01_HARDWARE_INTERFACE_HPP

#include <atomic>
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#ifdef GRASP01_HARDWARE__THREAD_PRIORITY
  #include <pthread.h>
  #include <sched.h>
#endif

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <myactuator_rmd/driver/can_driver.hpp>
#include <myactuator_rmd/actuator_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "grasp01_hardware/conversions.hpp"
#include "grasp01_hardware/visibility_control.hpp"

namespace grasp01_hardware {

class Grasp01HardwareInterface : public hardware_interface::SystemInterface {
public:
    Grasp01HardwareInterface() = default;
    ~Grasp01HardwareInterface() override;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    GRASP01_HARDWARE_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    GRASP01_HARDWARE_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    GRASP01_HARDWARE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    GRASP01_HARDWARE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
    static rclcpp::Logger getLogger();

    void asyncThread(std::chrono::milliseconds const& cycle_time);

    [[nodiscard]]
    bool startAsyncThread(std::chrono::milliseconds const& cycle_time);

    void stopAsyncThread();

    std::string ifname_;
    std::chrono::milliseconds cycle_time_;
    std::chrono::milliseconds timeout_;

    std::vector<std::uint32_t> actuator_ids_;
    std::vector<double> torque_constants_;
    std::vector<double> max_velocities_;
    std::vector<double> directions_;
    std::vector<std::string> joint_names_;

    std::unique_ptr<myactuator_rmd::CanDriver> driver_;
    std::vector<std::unique_ptr<myactuator_rmd::ActuatorInterface>> actuators_;
    std::array<myactuator_rmd::Feedback, 6> feedback_;

    std::array<std::atomic<double>, 6> async_position_states_;
    std::array<std::atomic<double>, 6> async_velocity_states_;
    std::array<std::atomic<double>, 6> async_effort_states_;
    std::array<std::atomic<double>, 6> async_position_commands_;
    std::array<std::atomic<double>, 6> async_velocity_commands_;
    std::array<std::atomic<double>, 6> async_effort_commands_;

    std::atomic<bool> position_interface_running_;
    std::atomic<bool> velocity_interface_running_;
    std::atomic<bool> effort_interface_running_;

    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> effort_states_;
    std::vector<double> position_commands_;
    std::vector<double> velocity_commands_;
    std::vector<double> effort_commands_;

    std::thread async_thread_;
    std::atomic<bool> stop_async_thread_;
};

}  // namespace grasp01_hardware

#endif  // GRASP01_HARDWARE_INTERFACE_HPP
