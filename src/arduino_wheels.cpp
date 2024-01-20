// Copyright (c) 2024, omar.salem
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <limits>
#include <vector>

#include "two_wheels/arduino_wheels.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace two_wheels {
    hardware_interface::CallbackReturn ArduinoWheels::on_init(
            const hardware_interface::HardwareInfo &info) {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoWheels"), "on_init ...please wait...");

        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArduinoWheels::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoWheels"), "on_configure ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    std::vector <hardware_interface::StateInterface> ArduinoWheels::export_state_interfaces() {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoWheels"), "export_state_interfaces ...please wait...");
        std::vector <hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }

        return state_interfaces;
    }

    std::vector <hardware_interface::CommandInterface> ArduinoWheels::export_command_interfaces() {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoWheels"), "export_command_interfaces ...please wait...");
        std::vector <hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ArduinoWheels::on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoWheels"), "on_activate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArduinoWheels::on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoWheels"), "on_deactivate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ArduinoWheels::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoWheels"), "read ...please wait...");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArduinoWheels::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoWheels"), "write ...please wait...");
        return hardware_interface::return_type::OK;
    }

}


PLUGINLIB_EXPORT_CLASS(
        two_wheels::ArduinoWheels, hardware_interface::SystemInterface
)
