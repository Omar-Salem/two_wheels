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
#include <array>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "angles/angles.h"

using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;
using map_msgs::msg::OccupancyGridUpdate;
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
                "/map", 10, std::bind(&Mapper::mapCallback, this, _1));

        poseSubscription_ = this->create_subscription<PoseWithCovarianceStamped>(
                "/pose", 10, std::bind(&Mapper::poseTopicCallback, this, _1));
        /*
         * TODO
         * rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr cancel_navto_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node,"/navigate_to_pose");
         * */
        goalPublisher_ = this->create_publisher<PoseStamped>(
                "/goal_pose", 10);

        controlLoopTimer_ = this->create_wall_timer(
                CONTROL_LOOP_INTERVAL_MILLI_SEC, [this] { controlLoop(); });
        sleep(10);//Wait till bt_navigator is active //TODO
    }

private:
    /*
# 0 represents unoccupied, 1 represents definitely occupied, and

# -1 represents unknown.
 */
    static constexpr signed char UNKNOWN = -1;
    set<pair<int, int>> visited;
    bool reachedGoal = false;
    bool isDone = false;
    rclcpp::TimerBase::SharedPtr controlLoopTimer_;
    static constexpr milliseconds CONTROL_LOOP_INTERVAL_MILLI_SEC = 500ms;

    rclcpp::Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    rclcpp::Subscription<OccupancyGridUpdate>::SharedPtr mapUpdatesSubscription_;

    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr poseSubscription_;
    rclcpp::Publisher<PoseStamped>::SharedPtr goalPublisher_;
    PoseStamped goal;

    uint32_t width;
    vector<signed char> map;
    double originX, originY;

    void mapCallback(OccupancyGrid::UniquePtr occupancyGrid) {
        width = occupancyGrid->info.width;
        map = occupancyGrid->data;
        const auto origin = occupancyGrid->info.origin.position;
        originX = origin.x;
        originY = origin.y;

        mapSubscription_.reset();//unsubscribe

        mapUpdatesSubscription_ = this->create_subscription<OccupancyGridUpdate>(
                "/map_updates", 10, std::bind(&Mapper::mapUpdatesCallback, this, _1));
    }

    void mapUpdatesCallback(OccupancyGridUpdate::UniquePtr occupancyGridUpdate) {
        width = occupancyGridUpdate->width;
        map = occupancyGridUpdate->data;
        originX = occupancyGridUpdate->x;
        originY = occupancyGridUpdate->y;
    }

    void poseTopicCallback(PoseWithCovarianceStamped::UniquePtr pose) {
        const auto goalPosition = goal.pose.position;
        const auto currentPosition = pose->pose.pose.position;
        reachedGoal = calculateDistance(goalPosition.x,
                                        goalPosition.y,
                                        currentPosition.x,
                                        currentPosition.y) <= 2;
        if (reachedGoal) {//TODO use nav2
            RCLCPP_INFO(this->get_logger(), "********************* GOAL REACHED *******************");
        }
    }

    pair<bool, pair<int, int>> findFrontier() {
        for (size_t i = 0; i < map.size(); ++i) {
            if (map[i] != UNKNOWN) {
                continue;
            }

            auto y = i / width;
            auto x = i - (y * width);
            x += originX;
            y += originY;

            const auto coords = make_pair(x, y);
            const bool notVisited = visited.find(coords) == visited.end();
            if (notVisited) {
                visited.insert(coords);
                return make_pair(true, coords);
            }
        }
        return make_pair(false, make_pair(0, 0));
    }

    void controlLoop() {
        if (map.empty() || !reachedGoal || isDone) {
            return;
        }
        auto frontierResult = findFrontier();
        isDone = !frontierResult.first;
        if (isDone) {
            RCLCPP_INFO(this->get_logger(),
                        "********************* No Frontier FOUND!!**********************************");
            //TODO save and
            saveMap("map");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "********************* MAP SIZE******************* : %zu", map.size());
        RCLCPP_INFO(this->get_logger(), "********************* MAP WIDTH ******************* : %u", width);

        const auto coords = frontierResult.second;
        const auto x = coords.first;
        const auto y = coords.second;
        RCLCPP_INFO(this->get_logger(), "********************* Navigating to : %d,%d", x, y);
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.orientation.z = .38;
        goal.pose.orientation.w = .92;
        goal.header.frame_id = "map";
        reachedGoal = false;
        goalPublisher_->publish(goal);
    }

    void saveMap(const std::string &mapName) {
        auto packageName = "two_wheels";//TODO get package name
        auto executionPath = std::filesystem::current_path().string();///home/${user}/ros2_ws
        auto mapsDir = executionPath + "/install/" + packageName + "/share/" + packageName + "/maps";
        RCLCPP_INFO(this->get_logger(), "********************* mapsDir %s *******************",
                    mapsDir.c_str());
        //TODO better to call
        /*
         *   auto map_saver = std::make_shared<nav2_map_server::MapSaver>();
    map_saver->on_configure(rclcpp_lifecycle::State());
    if (map_saver->saveMapTopicToFile(map_topic, save_parameters)) {
      retcode = 0;
    } else {
      retcode = 1;
    }
         * */
        auto command = "ros2 run nav2_map_server map_saver_cli -f " + mapsDir + "/" + mapName;
        auto returnCode = system(command.c_str());
//        auto returnCode = 0;
        if (returnCode != 0) {
            //TODO throw exception?
//            RCLCPP_ERROR(logger, "Unexpected problem appear: %s", e.what());
        }
    }

    static double calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapper>());
    rclcpp::shutdown();
    return 0;
}