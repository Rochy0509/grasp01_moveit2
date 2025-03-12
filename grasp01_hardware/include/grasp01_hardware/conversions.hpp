#ifndef GRASP01_HARDWARE_CONVERSIONS
#define GRASP01_HARDWARE_CONVERSIONS

#include <cmath>

namespace grasp01_hardware {

    constexpr double degToRad(double const degrees) noexcept {
        return degrees * (M_PI / 180.0);
    }

    constexpr double radToDeg(double const radians) noexcept {
        return radians * (180.0 / M_PI);
    }

    constexpr double currentToTorque(double const current, double const torque_constant) noexcept {
        return current * torque_constant;
    }

    constexpr double torqueToCurrent(double const torque, double const torque_constant) noexcept {
        return torque / torque_constant;
    }

}  // namespace grasp01_hardware

#endif  // GRASP01_HARDWARE_CONVERSIONS