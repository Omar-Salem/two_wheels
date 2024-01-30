//
// Created by omar on 1/30/24.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <time.h>

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
                500ms, std::bind(&BumpAndGo::controlLoop, this));
    }

private:
    bool turning_ = false;
    double startingAngle = 0;
    time_t startingTime;
    rclcpp::TimerBase::SharedPtr controlLoopTimer_;
    const milliseconds CONTROL_LOOP_INTERVAL_MILLISEC = 500ms;
    Range::UniquePtr range_;
    Odometry::UniquePtr odometry_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twistStampedPublisher_;
    rclcpp::Subscription<Range>::SharedPtr rangeTopicSubscription_;
    rclcpp::Subscription<Odometry>::SharedPtr odomTopicSubscription_;
    TwistStamped twistMsg;

    void rangeTopicCallback(Range::UniquePtr range) {
//        RCLCPP_INFO(this->get_logger(), "min_range: '%s'", to_string(range.min_range).c_str());
//        RCLCPP_INFO(this->get_logger(), "max_range: '%s'", to_string(range.max_range).c_str());
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

    void controlLoop() {
        if (range_ == nullptr || odometry_ == nullptr) {
            return;
        }
//        bool passedControlInterval =
//                abs(difftime(time(NULL), startingTime)) >= 1; //TODO use CONTROL_LOOP_INTERVAL_MILLISEC

        RCLCPP_INFO(this->get_logger(), "turning_: '%s'", to_string(turning_).c_str());
//        RCLCPP_INFO(this->get_logger(), "passedControlInterval: '%s'", to_string(passedControlInterval).c_str());
        if (turning_) {
            bool reachedAngle = abs(getCurrentYawInDegrees() - startingAngle) >= 90;
            RCLCPP_INFO(this->get_logger(), "reachedAngle: '%s'", to_string(reachedAngle).c_str());
            if (reachedAngle) {
                turning_ = false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "range: '%s'", to_string(range_->range).c_str());
            //Clear, keep going
            if (range_->range > 0.5) {
                twistMsg.twist.linear.x = 0.1;
                twistMsg.twist.linear.z = 0.0;
            } else {
                RCLCPP_INFO(this->get_logger(), "BLOCKED! Turning!.....");
                turning_ = true;
                startingAngle = getCurrentYawInDegrees();
                startingTime = time(NULL);
                //Turn left or right by random
                int lb = 1, ub = 100;
                auto chosen = (rand() % (ub - lb + 1)) + lb;

                twistMsg.twist.linear.x = 0.0;
                twistMsg.twist.linear.z = 1.57 * chosen <= 50 ? 1 : -1;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", to_string(twistMsg.twist.linear.z).c_str());
        twistStampedPublisher_->publish(twistMsg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BumpAndGo>());
    rclcpp::shutdown();
    return 0;
}