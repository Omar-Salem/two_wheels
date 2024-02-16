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
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "angles/angles.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;
using angles::to_degrees;
using angles::from_degrees;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;
using std::chrono::steady_clock;


class Mapper : public rclcpp::Node {
public:
    Mapper()
            : Node("mapper") {
        mapSubscription_ = this->create_subscription<OccupancyGrid>(
                "/global_costmap/costmap", 10, std::bind(&Mapper::mapTopicCallback, this, _1));
        poseSubscription_ = this->create_subscription<PoseWithCovarianceStamped>(
                "/pose", 10, std::bind(&Mapper::poseTopicCallback, this, _1));

        goalPublisher_ = this->create_publisher<PoseStamped>(
                "/goal_pose", 10);

        controlLoopTimer_ = this->create_wall_timer(
                CONTROL_LOOP_INTERVAL_MILLI_SEC, [this] { controlLoop(); });
        sleep(5);//Wait till bt_navigator is active
    }

private:
    static constexpr signed char UNKNOWN = -1;
    set<pair<double, double>> visited;
    bool reachedGoal = false;
    bool isDone = false;
    rclcpp::TimerBase::SharedPtr controlLoopTimer_;
    static constexpr milliseconds CONTROL_LOOP_INTERVAL_MILLI_SEC = 500ms;
    OccupancyGrid::UniquePtr occupancyGrid_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr poseSubscription_;
    rclcpp::Publisher<PoseStamped>::SharedPtr goalPublisher_;
    PoseStamped goal;

    void mapTopicCallback(OccupancyGrid::UniquePtr occupancyGrid) {
        occupancyGrid_ = std::move(occupancyGrid);
    }

    void poseTopicCallback(PoseWithCovarianceStamped::UniquePtr pose) {
        const auto goalPosition = goal.pose.position;
        const auto currentPosition = pose->pose.pose.position;
        reachedGoal = calculateDistance(goalPosition.x,
                                        goalPosition.y,
                                        currentPosition.x,
                                        currentPosition.y) <= 5;
    }

    void controlLoop() {
        if (occupancyGrid_ == nullptr || !reachedGoal) {
            return;
        }
        const auto metaData = occupancyGrid_->info;
        const auto width = metaData.width;
//        const auto height = metaData.height;
        /*
        # 0 represents unoccupied, 1 represents definitely occupied, and

        # -1 represents unknown.
         */
        const auto map = occupancyGrid_->data;
        RCLCPP_INFO(this->get_logger(), "********************* MAP SIZE******************* : %zu", map.size());

        auto iterator = std::find_if(map.cbegin(),
                                     map.cend(),
                                     [](auto cell) { return cell == UNKNOWN; });
        isDone = map.cend() == iterator;
        if (isDone) {
            RCLCPP_INFO(this->get_logger(),
                        "**************************** MAPPING COMPLETE ****************************");
            return;
        }

        const auto val = iterator - map.begin();
        RCLCPP_INFO(this->get_logger(), "********************* FIRST FRONTIER INDEX******************* : %td", val);
        RCLCPP_INFO(this->get_logger(), "********************* WIDTH ******************* : %u", width);

        auto y = val / width;
        auto x = val - (y * width);
        const auto origin = metaData.origin.position;
        x += origin.x;
        y += origin.y;

        RCLCPP_INFO(this->get_logger(), "********************* COORDS ******************* : %td,%td", x, y);

        auto coords = make_pair(x, y);
        bool notVisited = visited.find(coords) == visited.end();
        if (notVisited) {
            visited.insert(coords);
            reachedGoal = true;
            RCLCPP_INFO(this->get_logger(), "********************* Navigating to : %td,%td", x, y);
            goal.pose.position.x = x;
            goal.pose.position.y = y;
//            goal.pose.orientation.z = .38;
//            goal.pose.orientation.w = .92;
            goal.header.frame_id = "map";
            goalPublisher_->publish(goal);
        } else {
            RCLCPP_INFO(this->get_logger(), "********************* VISITED??????? ******************* : %td,%td", x, y);
        }
    }

    double calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapper>());
    rclcpp::shutdown();
    return 0;
}