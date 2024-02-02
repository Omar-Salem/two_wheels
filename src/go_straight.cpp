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
#include "behaviortree_cpp_v3/bt_factory.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "angles/angles.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
    class GoStraight : public BT::ActionNodeBase {
    public:
        GoStraight(
                const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf)
                : BT::ActionNodeBase(xml_tag_name, conf) {
            config().blackboard->get("node", node_);
            twistStampedPublisher_ = node_->create_publisher<TwistStamped>(
                    "/diff_drive_controller/cmd_vel", 10);
        }

        static BT::PortsList providedPorts() {
            return BT::PortsList(
                    {
                    });
        }


        BT::NodeStatus tick() override {
            TwistStamped twistMsg;
            twistMsg.twist.linear.x = LINEAR_VELOCITY;
            twistStampedPublisher_->publish(twistMsg);
            return BT::NodeStatus::SUCCESS;
        }

        void halt() override {
        }

    private:
        static constexpr double LINEAR_VELOCITY = 0.1;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<TwistStamped>::SharedPtr twistStampedPublisher_;

    };
}

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<bt::GoStraight>("GoStraight");
}