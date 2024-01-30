//
// Created by omar on 1/30/24.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::TwistStamped;
using std::to_string;
using namespace std::chrono_literals;


class BumpAndGo : public rclcpp::Node {
public:
    BumpAndGo()
            : Node("bump_go") {
        twistStampedPublisher_ = this->create_publisher<TwistStamped>(
                "/diff_drive_controller/cmd_vel", 10);
        rangeTopicSubscription_ = this->create_subscription<Range>(
                "/two_wheels/range", 10, std::bind(&BumpAndGo::rangeTopicCallback, this, _1));
        controlLoopTimer_ = this->create_wall_timer(
                500ms, std::bind(&BumpAndGo::controlLoop, this));
    }

private:

    rclcpp::TimerBase::SharedPtr controlLoopTimer_;
    Range range_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twistStampedPublisher_;
    rclcpp::Subscription<Range>::SharedPtr rangeTopicSubscription_;

    void rangeTopicCallback(const Range &range) {
        RCLCPP_INFO(this->get_logger(), "min_range: '%s'", to_string(range.min_range).c_str());
        RCLCPP_INFO(this->get_logger(), "max_range: '%s'", to_string(range.max_range).c_str());
        RCLCPP_INFO(this->get_logger(), "range: '%s'", to_string(range.range).c_str());
        range_ = range;
    }

    void controlLoop() {
//        if (&range_ == nullptr) {
//            return;
//        }
        auto message = TwistStamped();
        message.twist.linear.x = 0.7;
        message.twist.linear.z = 1.0;
//        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        twistStampedPublisher_->publish(message);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BumpAndGo>());
    rclcpp::shutdown();
    return 0;
}