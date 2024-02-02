//
// Created by omar on 2/2/24.
//
#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using sensor_msgs::msg::Range;
using std::placeholders::_1;
namespace bt {
    class IsClear : public BT::ConditionNode {
    public:
        IsClear(
                const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf)
                : BT::ConditionNode(xml_tag_name, conf) {
            config().blackboard->get("node", node_);
            rangeTopicSubscription_
                    = node_->create_subscription<Range>(
                    "/two_wheels/range", 10, std::bind(&IsClear::rangeTopicCallback, this, _1));
        }

        static BT::PortsList providedPorts() {
            return BT::PortsList(
                    {
                    });
        }


        BT::NodeStatus tick() override {
            if (range_ == nullptr) {
                return BT::NodeStatus::FAILURE;
            }

            if (range_->range >= 0.5) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        Range::UniquePtr range_;
        rclcpp::Subscription<Range>::SharedPtr rangeTopicSubscription_;

        void rangeTopicCallback(Range::UniquePtr range) {
            range_ = std::move(range);
        }

    };
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<bt::IsClear>("IsClear");
}
