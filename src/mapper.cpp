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
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "angles/angles.h"

using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;
using map_msgs::msg::OccupancyGridUpdate;
using nav2_costmap_2d::Costmap2D;
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
                "/map", 10, std::bind(&Mapper::updateFullMap, this, _1));

        mapUpdatesSubscription_ = this->create_subscription<OccupancyGridUpdate>(
                "/map_updates", 10, std::bind(&Mapper::updatePartialMap, this, _1));


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
    Costmap2D costmap_;
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

    std::array<unsigned char, 256> init_translation_table() {
        std::array<unsigned char, 256> cost_translation_table;

        // lineary mapped from [0..100] to [0..255]
        for (size_t i = 0; i < 256; ++i) {
            cost_translation_table[i] =
                    static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
        }

        // special values:
        cost_translation_table[0] = 0;      // NO obstacle
        cost_translation_table[99] = 253;   // INSCRIBED obstacle
        cost_translation_table[100] = 254;  // LETHAL obstacle
        cost_translation_table[static_cast<unsigned char>(-1)] = 255;  // UNKNOWN

        return cost_translation_table;
    }

    std::array<unsigned char, 256> cost_translation_table_ = init_translation_table();

    void updateFullMap(OccupancyGrid::UniquePtr msg) {
        unsigned int size_in_cells_x = msg->info.width;
        unsigned int size_in_cells_y = msg->info.height;
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        RCLCPP_INFO(this->get_logger(), "received full new map, resizing to: %d, %d", size_in_cells_x,
                    size_in_cells_y);
        costmap_.resizeMap(size_in_cells_x,
                           size_in_cells_y,
                           resolution,
                           origin_x,
                           origin_y);

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        std::lock_guard<Costmap2D::mutex_t> lock(*mutex);

        // fill map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCLCPP_INFO(this->get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
            unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
            costmap_data[i] = cost_translation_table_[cell_cost];
        }
    }

    void updatePartialMap(OccupancyGridUpdate::UniquePtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "received partial map update");
        if (msg->x < 0 || msg->y < 0) {
            RCLCPP_ERROR(this->get_logger(), "negative coordinates, invalid update. x: %d, y: %d", msg->x,
                         msg->y);
            return;
        }

        size_t x0 = static_cast<size_t>(msg->x);
        size_t y0 = static_cast<size_t>(msg->y);
        size_t xn = msg->width + x0;
        size_t yn = msg->height + y0;

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        std::lock_guard<Costmap2D::mutex_t> lock(*mutex);

        size_t costmap_xn = costmap_.getSizeInCellsX();
        size_t costmap_yn = costmap_.getSizeInCellsY();

        if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
            y0 > costmap_yn) {
            RCLCPP_WARN(this->get_logger(), "received update doesn't fully fit into existing map, "
                                            "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
                                            "map is: [0, %lu], [0, %lu]",
                        x0, xn, y0, yn, costmap_xn, costmap_yn);
        }

        // update map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t i = 0;
        for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
            for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
                size_t idx = costmap_.getIndex(x, y);
                unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
                costmap_data[idx] = cost_translation_table_[cell_cost];
                ++i;
            }
        }
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