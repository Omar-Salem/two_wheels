// Copyright (c) 2024, omar.salem
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <limits>
#include <vector>

#include "Arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace diff_drive_hw_interface {
    CallbackReturn Arduino::on_init(
            const HardwareInfo &info) {
        RCLCPP_INFO(get_logger("Arduino"), "on_init ...please wait...");

        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        leftWheel = new Wheel("left_wheel_joint"); //TODO use smart pointers!
        rightWheel = new Wheel("right_wheel_joint");
        firmware = new ArduinoFirmware();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Arduino::on_configure(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("Arduino"), "on_configure ...please wait...");
        firmware->configure();
        return CallbackReturn::SUCCESS;
    }

    vector<StateInterface> Arduino::export_state_interfaces() {
        RCLCPP_INFO(get_logger("Arduino"), "export_state_interfaces ...please wait...");
        vector<StateInterface> state_interfaces;

        state_interfaces.emplace_back(
                leftWheel->name, HW_IF_POSITION, &leftWheel->position_state);

        state_interfaces.emplace_back(
                leftWheel->name, HW_IF_VELOCITY, &leftWheel->velocity_state);


        state_interfaces.emplace_back(
                rightWheel->name, HW_IF_POSITION, &rightWheel->position_state);

        state_interfaces.emplace_back(
                rightWheel->name, HW_IF_VELOCITY, &rightWheel->velocity_state);

        return state_interfaces;
    }

    vector<CommandInterface> Arduino::export_command_interfaces() {
        RCLCPP_INFO(get_logger("Arduino"), "export_command_interfaces ...please wait...");
        vector<CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
                leftWheel->name, HW_IF_VELOCITY, &leftWheel->velocity_command);
        command_interfaces.emplace_back(
                rightWheel->name, HW_IF_VELOCITY, &rightWheel->velocity_command);


        return command_interfaces;
    }

    CallbackReturn Arduino::on_activate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("Arduino"), "on_activate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Arduino::on_deactivate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("Arduino"), "on_deactivate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    return_type Arduino::read(
            const Time & /*time*/, const Duration & /*period*/) {
        RCLCPP_INFO(get_logger("Arduino"), "read ...please wait...");

        //TODO:update leftWheel->position_state and rightWheel->position_state from arduino
        //TODO:update leftWheel->velocity_state and rightWheel->velocity_state from arduino

        firmware->getFirstMotorPosition();
        return return_type::OK;
    }

    return_type Arduino::write(
            const Time & /*time*/, const Duration & /*period*/) {
        RCLCPP_INFO(get_logger("Arduino"), "write ...please wait...");
        RCLCPP_INFO(get_logger("Arduino"), "leftWheel->velocity_command: %f'",
                    leftWheel->velocity_command);
        RCLCPP_INFO(get_logger("Arduino"), "rightWheel->velocity_command: %f'",
                    rightWheel->velocity_command);
        //TODO:send leftWheel->velocity_command and rightWheel->velocity_command to firmware
        firmware->setFirstMotorVelocity(12.0);
//        firmware->setFirstMotorVelocity(leftWheel->velocity_command);
        return return_type::OK;
    }

}


PLUGINLIB_EXPORT_CLASS(diff_drive_hw_interface::Arduino, SystemInterface)
