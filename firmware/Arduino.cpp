// Copyright (c) 2024, omar.salem
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <limits>
#include <vector>

#include "diff_drive_hw_interface/Arduino.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace diff_drive_hw_interface {
    hardware_interface::CallbackReturn Arduino::on_init(
            const hardware_interface::HardwareInfo &info) {
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "on_init ...please wait...");

        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        leftWheel = new Wheel("left_wheel_joint");
        rightWheel = new Wheel("right_wheel_joint");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Arduino::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "on_configure ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Arduino::export_state_interfaces() {
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "export_state_interfaces ...please wait...");
        std::vector<hardware_interface::StateInterface> state_interfaces;
//        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "info_.joints[0].name: %s'",info_.joints[0].name.c_str());
//        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "info_.joints[1].name: %s'",info_.joints[1].name.c_str());


        state_interfaces.emplace_back(
                leftWheel->name, hardware_interface::HW_IF_POSITION, &leftWheel->position_state);

        state_interfaces.emplace_back(
                leftWheel->name, hardware_interface::HW_IF_VELOCITY, &leftWheel->velocity_state);


        state_interfaces.emplace_back(
                rightWheel->name, hardware_interface::HW_IF_POSITION, &rightWheel->position_state);

        state_interfaces.emplace_back(
                rightWheel->name, hardware_interface::HW_IF_VELOCITY, &rightWheel->velocity_state);

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Arduino::export_command_interfaces() {
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "export_command_interfaces ...please wait...");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
                leftWheel->name, hardware_interface::HW_IF_VELOCITY, &leftWheel->velocity_command);
        command_interfaces.emplace_back(
                rightWheel->name, hardware_interface::HW_IF_VELOCITY, &rightWheel->velocity_command);


        return command_interfaces;
    }

    hardware_interface::CallbackReturn Arduino::on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "on_activate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Arduino::on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "on_deactivate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Arduino::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "read ...please wait...");
        //TODO:update leftWheel->position_state and rightWheel->position_state from arduino
        //TODO:update leftWheel->velocity_state and rightWheel->velocity_state from arduino
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Arduino::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "write ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "leftWheel->velocity_command: %f'",
                    leftWheel->velocity_command);
        RCLCPP_INFO(rclcpp::get_logger("Arduino"), "rightWheel->velocity_command: %f'",
                    rightWheel->velocity_command);
        //TODO:send leftWheel->velocity_command and rightWheel->velocity_command to firmware
        return hardware_interface::return_type::OK;
    }

}


PLUGINLIB_EXPORT_CLASS(diff_drive_hw_interface::Arduino, hardware_interface::SystemInterface)
