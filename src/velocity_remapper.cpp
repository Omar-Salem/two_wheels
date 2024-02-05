//
// Created by omar on 2/5/24.
//
//
// Created by omar on 1/30/24.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <time.h>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "angles/angles.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using angles::to_degrees;
using angles::from_degrees;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;


class VelocityReMapper : public rclcpp::Node {
public:
    VelocityReMapper()
            : Node("velocity_remapper") {
        twistPublisher_ = this->create_publisher<Twist>(
                "/diff_drive_controller/cmd_vel_unstamped", 10);
        velocityTopicSubscription = this->create_subscription<Twist>(
                "/cmd_vel", 10, std::bind(&VelocityReMapper::velocityTopicCallback, this, _1));
    }

private:
    rclcpp::Publisher<Twist>::SharedPtr twistPublisher_;
    rclcpp::Subscription<Twist>::SharedPtr velocityTopicSubscription;

    void velocityTopicCallback(Twist::UniquePtr twistMsg) {
        twistPublisher_->publish(std::move(twistMsg));
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityReMapper>());
    rclcpp::shutdown();
    return 0;
}