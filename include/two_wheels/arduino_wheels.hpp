// Copyright (c) 2024, omar.salem
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef TWO_WHEELS__ARDUINO_WHEELS_HPP_
#define TWO_WHEELS__ARDUINO_WHEELS_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "two_wheels/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "Wheel.h"

namespace two_wheels {
    class ArduinoWheels : public hardware_interface::SystemInterface {
    public:
        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo &info) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State &previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State &previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State &previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::return_type read(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::return_type write(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        Wheel *leftWheel;
        Wheel *rightWheel;
    };

}  // namespace two_wheels

#endif  // TWO_WHEELS__ARDUINO_WHEELS_HPP_
