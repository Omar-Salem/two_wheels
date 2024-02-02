//
// Created by omar on 2/2/24.
//

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <time.h>
#include <random>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "angles/angles.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

using std::placeholders::_1;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using angles::to_degrees;
using angles::from_degrees;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;

using sensor_msgs::msg::Range;
namespace bt {
    class Turn : public BT::ActionNodeBase {
    public:
        Turn(const std::string &xml_tag_name,
             const BT::NodeConfiguration &conf)
                : BT::ActionNodeBase(xml_tag_name, conf) {
            config().blackboard->get("node", node_);
            twistStampedPublisher_ = node_->create_publisher<TwistStamped>(
                    "/diff_drive_controller/cmd_vel", 10);
            odomTopicSubscription_ = node_->create_subscription<Odometry>(
                    "/diff_drive_controller/odom", 10, std::bind(&Turn::odomTopicCallback, this, _1));
        }

        static BT::PortsList providedPorts() {
            return BT::PortsList(
                    {
                            BT::InputPort<double>("angle")
                    });
        }


        BT::NodeStatus tick() override {
            TwistStamped twistMsg;
            if (status() == BT::NodeStatus::IDLE) {
                twistMsg.twist.linear.x = 0.0;
                twistMsg.twist.angular.z = 1.57 * getRandomDirection();
            }
            double startingYaw = 0;
            getInput("angle", startingYaw);
            double yaw = getYaw();
            bool reachedAngle = abs(yaw - startingYaw) > 1.5708; //90 degree turn
            if (reachedAngle) {
                return BT::NodeStatus::SUCCESS;
            }
            twistStampedPublisher_->publish(twistMsg);
            return BT::NodeStatus::RUNNING;
        }

        void halt() override {
        }

    private:
        rclcpp::Node::SharedPtr node_;
        Odometry::UniquePtr odometry_;
        rclcpp::Publisher<TwistStamped>::SharedPtr twistStampedPublisher_;
        rclcpp::Subscription<Odometry>::SharedPtr odomTopicSubscription_;

        void odomTopicCallback(Odometry::UniquePtr odometry) {
            odometry_ = std::move(odometry);
        }

        double getYaw() {
            tf2::Quaternion quat_tf;
            geometry_msgs::msg::Quaternion quat_msg = odometry_->pose.pose.orientation;
            tf2::fromMsg(quat_msg, quat_tf);
            tf2::Matrix3x3 m(quat_tf);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            return yaw;
        }

        static int getRandomDirection() {
            random_device rd;
            mt19937 mt(rd());
            uniform_int_distribution<int> dist(1, 100);
            auto chosen = dist(mt);
            return chosen <= 50 ? 1 : -1;
        }


    };
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<bt::Turn>("Turn");
}