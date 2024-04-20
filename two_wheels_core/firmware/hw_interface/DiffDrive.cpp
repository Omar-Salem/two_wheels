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
                    this->readOdom(motorsOdom);
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
        frontLeftWheel = make_unique<Wheel>("front_left_wheel_joint");
        frontRightWheel = make_unique<Wheel>("front_right_wheel_joint");
        rearLeftWheel = make_unique<Wheel>("rear_left_wheel_joint");
        rearRightWheel = make_unique<Wheel>("rear_right_wheel_joint");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DiffDrive::on_configure(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger("DiffDrive"), "on_configure ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    vector <StateInterface> DiffDrive::export_state_interfaces() {
        RCLCPP_INFO(get_logger("DiffDrive"), "export_state_interfaces ...please wait...");
        vector <StateInterface> state_interfaces;

//        frontLeftWheel
        state_interfaces.emplace_back(
                frontLeftWheel->name, HW_IF_POSITION, &frontLeftWheel->position_state);

        state_interfaces.emplace_back(
                frontLeftWheel->name, HW_IF_VELOCITY, &frontLeftWheel->velocity_state);

//        frontRightWheel
        state_interfaces.emplace_back(
                frontRightWheel->name, HW_IF_POSITION, &frontRightWheel->position_state);

        state_interfaces.emplace_back(
                frontRightWheel->name, HW_IF_VELOCITY, &frontRightWheel->velocity_state);

//        rearLeftWheel
        state_interfaces.emplace_back(
                rearLeftWheel->name, HW_IF_POSITION, &rearLeftWheel->position_state);

        state_interfaces.emplace_back(
                rearLeftWheel->name, HW_IF_VELOCITY, &rearLeftWheel->velocity_state);

//        rearRightWheel
        state_interfaces.emplace_back(
                rearRightWheel->name, HW_IF_POSITION, &rearRightWheel->position_state);

        state_interfaces.emplace_back(
                rearRightWheel->name, HW_IF_VELOCITY, &rearRightWheel->velocity_state);

        return state_interfaces;
    }

    vector <CommandInterface> DiffDrive::export_command_interfaces() {
        RCLCPP_INFO(get_logger("DiffDrive"), "export_command_interfaces ...please wait...");
        vector <CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
                frontLeftWheel->name, HW_IF_VELOCITY, &frontLeftWheel->velocity_command);
        command_interfaces.emplace_back(
                frontRightWheel->name, HW_IF_VELOCITY, &frontRightWheel->velocity_command);
        command_interfaces.emplace_back(
                rearLeftWheel->name, HW_IF_VELOCITY, &rearLeftWheel->velocity_command);
        command_interfaces.emplace_back(
                rearRightWheel->name, HW_IF_VELOCITY, &rearRightWheel->velocity_command);

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
        setMotorsVelocity(0, 0, 0, 0);
        return CallbackReturn::SUCCESS;
    }

    return_type DiffDrive::read(
            const Time & /*time*/, const Duration & /*period*/) {
        rclcpp::spin_some(node_);
        return return_type::OK;
    }

    return_type DiffDrive::write(
            const Time & /*time*/, const Duration & /*period*/) {
        setMotorsVelocity(frontLeftWheel->velocity_command,
                          frontRightWheel->velocity_command,
                          rearLeftWheel->velocity_command,
                          rearRightWheel->velocity_command);
        return return_type::OK;
    }

    void DiffDrive::setMotorsVelocity(double frontLeft,
                                      double frontRight,
                                      double rearLeft,
                                      double rearRight) {
        auto cmd_msg = std::make_shared<two_wheels_interfaces::msg::MotorsOdom>();
        cmd_msg->front_left.velocity = frontLeft;
        cmd_msg->front_right.velocity = frontRight;
        cmd_msg->rear_left.velocity = rearLeft;
        cmd_msg->rear_right.velocity = rearRight;
        velocityPublisher->publish(*cmd_msg);
    }

    void DiffDrive::readOdom(const two_wheels_interfaces::msg::MotorsOdom::SharedPtr motorsOdom) {
        frontLeftWheel->position_state = motorsOdom->front_left.position;
        frontLeftWheel->velocity_state = motorsOdom->front_left.velocity;

        frontRightWheel->position_state = motorsOdom->front_right.position;
        frontRightWheel->velocity_state = motorsOdom->front_right.velocity;

        rearLeftWheel->position_state = motorsOdom->rear_left.position;
        rearLeftWheel->velocity_state = motorsOdom->rear_left.velocity;

        rearRightWheel->position_state = motorsOdom->rear_right.position;
        rearRightWheel->velocity_state = motorsOdom->rear_right.velocity;
    }

}


PLUGINLIB_EXPORT_CLASS(hw_interface::DiffDrive, SystemInterface
)
