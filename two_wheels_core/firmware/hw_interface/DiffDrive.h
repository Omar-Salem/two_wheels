#ifndef DIFF_DRIVE__ARDUINO_WHEELS_HPP_
#define DIFF_DRIVE__ARDUINO_WHEELS_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "Wheel.h"

#include "two_wheels_interfaces/msg/motor.hpp"
#include "two_wheels_interfaces/msg/motors_odom.hpp"

using namespace hardware_interface;
using namespace rclcpp_lifecycle;
using namespace rclcpp;
using namespace std;
namespace hw_interface {
    class DiffDrive : public SystemInterface {
    public:
        DiffDrive();

        CallbackReturn on_init(
                const HardwareInfo &info) override;


        CallbackReturn on_configure(
                const State &previous_state) override;


        vector<StateInterface> export_state_interfaces() override;


        vector<CommandInterface> export_command_interfaces() override;


        CallbackReturn on_activate(
                const State &previous_state) override;


        CallbackReturn on_deactivate(
                const State &previous_state) override;


        return_type read(
                const Time &time, const Duration &period) override;


        return_type write(
                const Time &time, const Duration &period) override;

    private:
        unique_ptr<Wheel> leftWheel;
        unique_ptr<Wheel> rightWheel;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Subscription<two_wheels_interfaces::msg::MotorsOdom>::SharedPtr odomSubscription;
        rclcpp::Publisher<two_wheels_interfaces::msg::MotorsOdom>::SharedPtr velocityPublisher;

        void setMotorsVelocity(double m1, double m2);

        void readOdom(const two_wheels_interfaces::msg::MotorsOdom::SharedPtr motorsOdom);

    };

}

#endif  
