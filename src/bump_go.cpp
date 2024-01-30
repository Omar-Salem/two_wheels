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
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using angles::to_degrees;
using angles::from_degrees;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;


class BumpAndGo : public rclcpp::Node {
public:
    BumpAndGo()
            : Node("bump_go") {
        twistStampedPublisher_ = this->create_publisher<TwistStamped>(
                "/diff_drive_controller/cmd_vel", 10);
        rangeTopicSubscription_ = this->create_subscription<Range>(
                "/two_wheels/range", 10, std::bind(&BumpAndGo::rangeTopicCallback, this, _1));
        odomTopicSubscription_ = this->create_subscription<Odometry>(
                "/diff_drive_controller/odom", 10, std::bind(&BumpAndGo::odomTopicCallback, this, _1));
        controlLoopTimer_ = this->create_wall_timer(
                CONTROL_LOOP_INTERVAL_MILLI_SEC, std::bind(&BumpAndGo::controlLoop, this));
    }

private:
    bool turning_ = false;
    double startingAngle = 0;
    rclcpp::TimerBase::SharedPtr controlLoopTimer_;
    const milliseconds CONTROL_LOOP_INTERVAL_MILLI_SEC = 500ms;
    Range::UniquePtr range_;
    Odometry::UniquePtr odometry_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twistStampedPublisher_;
    rclcpp::Subscription<Range>::SharedPtr rangeTopicSubscription_;
    rclcpp::Subscription<Odometry>::SharedPtr odomTopicSubscription_;
    TwistStamped twistMsg;

    void rangeTopicCallback(Range::UniquePtr range) {
        range_ = std::move(range);
    }

    void odomTopicCallback(Odometry::UniquePtr odometry) {
        odometry_ = std::move(odometry);
    }

    double getCurrentYawInDegrees() {
        tf2::Quaternion quat_tf;
        geometry_msgs::msg::Quaternion quat_msg = odometry_->pose.pose.orientation;
        tf2::fromMsg(quat_msg, quat_tf);
        tf2::Matrix3x3 m(quat_tf);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return to_degrees(yaw);
    }

    int getRandomDirection() {
        random_device rd;
        mt19937 mt(rd());
        uniform_int_distribution<int> dist(1, 100);
        auto chosen = dist(mt);
        return chosen <= 50 ? 1 : -1;
    }

    void controlLoop() {
        if (range_ == nullptr || odometry_ == nullptr) {
            return;
        }

        if (turning_) {
            double currentYawInDegrees = getCurrentYawInDegrees();
            RCLCPP_INFO(this->get_logger(), "currentYawInDegrees: '%s'", to_string(currentYawInDegrees).c_str());
            bool reachedAngle = abs(currentYawInDegrees - startingAngle) >= 90;
            RCLCPP_INFO(this->get_logger(), "reachedAngle: '%s'", to_string(reachedAngle).c_str());
            if (reachedAngle) {
                turning_ = false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "range: '%s'", to_string(range_->range).c_str());
            //Clear, keep going
            if (range_->range > 0.5) {
                twistMsg.twist.linear.x = 0.1;
                twistMsg.twist.angular.z = 0.0;
            } else {
                RCLCPP_INFO(this->get_logger(), "BLOCKED! Turning!.....");
                turning_ = true;
                startingAngle = getCurrentYawInDegrees();
                twistMsg.twist.linear.x = 0.0;
                twistMsg.twist.angular.z = 1.57 * getRandomDirection();
            }
        }
        twistStampedPublisher_->publish(twistMsg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BumpAndGo>());
    rclcpp::shutdown();
    return 0;
}