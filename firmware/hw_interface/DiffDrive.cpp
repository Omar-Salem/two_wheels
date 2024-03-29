// Copyright (c) 2024, omar.salem
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <limits>
#include <vector>

#include "DiffDrive.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace hw_interface {
    CallbackReturn DiffDrive::on_init(
            const HardwareInfo &info) {
        RCLCPP_INFO(get_logger("DiffDrive"), "on_init ...please wait...");

        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        leftWheel = make_unique<Wheel>("left_wheel_joint");
        rightWheel = make_unique<Wheel>("right_wheel_joint");
        firmware = make_unique<ArduinoFirmware>();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffDrive::on_configure(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("DiffDrive"), "on_configure ...please wait...");
        firmware->connect();
        return CallbackReturn::SUCCESS;
    }

    vector<StateInterface> DiffDrive::export_state_interfaces() {
        RCLCPP_INFO(get_logger("DiffDrive"), "export_state_interfaces ...please wait...");
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

    vector<CommandInterface> DiffDrive::export_command_interfaces() {
        RCLCPP_INFO(get_logger("DiffDrive"), "export_command_interfaces ...please wait...");
        vector<CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
                leftWheel->name, HW_IF_VELOCITY, &leftWheel->velocity_command);
        command_interfaces.emplace_back(
                rightWheel->name, HW_IF_VELOCITY, &rightWheel->velocity_command);


        return command_interfaces;
    }

    CallbackReturn DiffDrive::on_activate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("DiffDrive"), "on_activate ...please wait...");
        firmware->ping();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffDrive::on_deactivate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("DiffDrive"), "on_deactivate ...please wait...");
        firmware->setMotorsVelocity(0, 0);
        firmware->disconnect();
        return CallbackReturn::SUCCESS;
    }

    return_type DiffDrive::read(
            const Time & /*time*/, const Duration & /*period*/) {
        RCLCPP_INFO(get_logger("DiffDrive"), "Reading ...please wait...");
        const MotorsOdom &odom = firmware->getMotorsOdom();
        leftWheel->position_state = odom.p1;
        leftWheel->velocity_state = odom.v1;

        rightWheel->position_state = odom.p2;
        rightWheel->velocity_state = odom.v2;

        RCLCPP_INFO(get_logger("DiffDrive"),
                    "leftWheel position:%f,rightWheel position:%f,leftWheel velocity:%f,rightWheel velocity:%f",
                    leftWheel->position_state, rightWheel->position_state,
                    leftWheel->velocity_state, rightWheel->velocity_state);

        return return_type::OK;
    }

    return_type DiffDrive::write(
            const Time & /*time*/, const Duration & /*period*/) {
//        RCLCPP_INFO(get_logger("DiffDrive"), "write ...please wait!!!!!!!...");
        RCLCPP_INFO(get_logger("DiffDrive"), "leftWheel->velocity_command: %f'",
                    leftWheel->velocity_command);
        RCLCPP_INFO(get_logger("DiffDrive"), "rightWheel->velocity_command: %f'",
                    rightWheel->velocity_command);

        firmware->setMotorsVelocity(leftWheel->velocity_command, rightWheel->velocity_command);
        return return_type::OK;
    }

}


PLUGINLIB_EXPORT_CLASS(hw_interface::DiffDrive, SystemInterface)
