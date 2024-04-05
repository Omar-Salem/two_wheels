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
    DiffDrive::DiffDrive() : node_(std::make_shared<rclcpp::Node>("two_wheels_motors_hw_interface_node")) {
        odomSubscription = node_->create_subscription<two_wheels_interfaces::msg::MotorsOdom>(
                "two_wheels/motors_state", 10,
                [this](const two_wheels_interfaces::msg::MotorsOdom::SharedPtr motorsOdom) {
                    leftWheel->position_state = motorsOdom->m1.position;
                    leftWheel->velocity_state = motorsOdom->m1.velocity;

                    rightWheel->position_state = motorsOdom->m2.position;
                    rightWheel->velocity_state = motorsOdom->m2.velocity;
                });
        velocityPublisher = node_->create_publisher<two_wheels_interfaces::msg::MotorsOdom>("two_wheels/motors_cmd",
                                                                                            10);
    }

    CallbackReturn DiffDrive::on_init(
            const HardwareInfo &info) {
        RCLCPP_INFO(get_logger("DiffDrive"), "on_init ...please wait...");

        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        leftWheel = make_unique<Wheel>("left_wheel_joint");
        rightWheel = make_unique<Wheel>("right_wheel_joint");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffDrive::on_configure(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("DiffDrive"), "on_configure ...please wait...");
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
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffDrive::on_deactivate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("DiffDrive"), "on_deactivate ...please wait...");
        setMotorsVelocity(0, 0);
        return CallbackReturn::SUCCESS;
    }

    return_type DiffDrive::read(
            const Time & /*time*/, const Duration & /*period*/) {
        RCLCPP_INFO(get_logger("DiffDrive"), "Reading ...please wait...");

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
        setMotorsVelocity(leftWheel->velocity_command, rightWheel->velocity_command);
        return return_type::OK;
    }

    void DiffDrive::setMotorsVelocity(double m1, double m2) {
        auto cmd_msg = std::make_shared<two_wheels_interfaces::msg::MotorsOdom>();
        cmd_msg->m1.velocity = m1;
        cmd_msg->m2.velocity = m2;
        velocityPublisher->publish(*cmd_msg);
    }

}


PLUGINLIB_EXPORT_CLASS(hw_interface::DiffDrive, SystemInterface)
