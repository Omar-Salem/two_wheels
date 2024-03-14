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

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/std_msgs/msg/color_rgba.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"

//#include <tf2/tf2/impl/

using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using nav_msgs::msg::OccupancyGrid;
using nav2_msgs::action::NavigateToPose;
using map_msgs::msg::OccupancyGridUpdate;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using std_msgs::msg::ColorRGBA;
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;
using namespace rclcpp;
using namespace rclcpp_action;
using namespace nav2_map_server;
using std::chrono::steady_clock;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Mapper : public Node {
public:
    Mapper()
            : Node("mapper") {
        mapSubscription_ = this->create_subscription<OccupancyGrid>(
                "/map", 10, bind(&Mapper::updateFullMap, this, _1));

        marker_array_publisher_ = this->create_publisher<MarkerArray>("/frontiers", 10);
        this->poseNavigator_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose");

        poseNavigator_->wait_for_action_server();
    }

private:
    Costmap2D costmap_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr poseNavigator_;
    Publisher<MarkerArray>::SharedPtr marker_array_publisher_;
    Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    bool isExploring = false;

    array<unsigned char, 256> init_translation_table() {
        array<unsigned char, 256> cost_translation_table{};

        // lineary mapped from [0..100] to [0..255]
        for (size_t i = 0; i < 256; ++i) {
            cost_translation_table[i] =
                    static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
        }

        // special values:
        cost_translation_table[0] = FREE_SPACE;
        cost_translation_table[99] = 253;
        cost_translation_table[100] = LETHAL_OBSTACLE;
        cost_translation_table[static_cast<unsigned char>(-1)] = NO_INFORMATION;

        return cost_translation_table;
    }

    array<unsigned char, 256> cost_translation_table_ = init_translation_table();

    void updateFullMap(OccupancyGrid::UniquePtr occupancyGrid) {
        const auto occupancyGridInfo = occupancyGrid->info;
        unsigned int size_in_cells_x = occupancyGridInfo.width;
        unsigned int size_in_cells_y = occupancyGridInfo.height;
        double resolution = occupancyGridInfo.resolution;
        double origin_x = occupancyGridInfo.origin.position.x;
        double origin_y = occupancyGridInfo.origin.position.y;

        RCLCPP_INFO(get_logger(), "received full new map, resizing to: %d, %d", size_in_cells_x,
                    size_in_cells_y);
        costmap_.resizeMap(size_in_cells_x,
                           size_in_cells_y,
                           resolution,
                           origin_x,
                           origin_y);

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        lock_guard<Costmap2D::mutex_t> lock(*mutex);

        // fill map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCLCPP_INFO(get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size(); ++i) {
            auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
            costmap_data[i] = cost_translation_table_[cell_cost];
        }

        explore();
    }

    void visualizeFrontiers(const Point &point) {
        ColorRGBA blue;
        blue.r = 0;
        blue.g = 0;
        blue.b = 1.0;
        blue.a = 1.0;
        ColorRGBA red;
        red.r = 1.0;
        red.g = 0;
        red.b = 0;
        red.a = 1.0;
        ColorRGBA green;
        green.r = 0;
        green.g = 1.0;
        green.b = 0;
        green.a = 1.0;

        MarkerArray markers_msg;
        vector<Marker> &markers = markers_msg.markers;
        Marker m;

        m.header.frame_id = "map";
        m.header.stamp = now();
        m.ns = "frontiers";
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 255;
        m.color.a = 255;
        // lives forever
        m.frame_locked = true;

        m.action = Marker::ADD;
        size_t id = 0;
        RCLCPP_INFO(get_logger(), "visualising %f,%f ", point.x, point.y);
        m.type = Marker::POINTS;
        m.id = int(id);
        m.scale.x = 0.1;
        m.scale.y = 0.1;
        m.scale.z = 0.1;

        m.color = blue;
        markers.push_back(m);
        ++id;
        m.type = Marker::SPHERE;
        m.id = int(id);
        m.pose.position = point;
        // scale frontier according to its cost (costier frontiers will be smaller)
        double scale = 1.0;
        m.scale.x = scale;
        m.scale.y = scale;
        m.scale.z = scale;
        m.points = {};
        m.color = green;
        markers.push_back(m);
        ++id;
        marker_array_publisher_->publish(markers_msg);
    }

    void clearFrontiers() {
        auto marker_array_msg = MarkerArray();
        auto marker = Marker();
        marker.id = 0;
        marker.ns = "frontiers";
        marker.action = Marker::DELETEALL;
        marker_array_msg.markers.push_back(marker);
        marker_array_publisher_->publish(marker_array_msg);
    }

    void stop() {
        RCLCPP_INFO(get_logger(), "Stopped...");
        poseNavigator_->async_cancel_all_goals();
        saveMap("/home/omar/ros2_ws/src/two_wheels/maps/apt");
    }

    void explore() {
        if (isExploring) { return; }
        auto target_position = findBoundary();
        if (!target_position.has_value()) {
            RCLCPP_WARN(get_logger(), "NO BOUNDARIES FOUND!!");
            stop();
            return;
        }
        visualizeFrontiers(target_position.value());
        auto goal = NavigateToPose::Goal();
        goal.pose.pose.position = target_position.value();
        goal.pose.pose.orientation.w = 1.;
        goal.pose.header.frame_id = "map";

        RCLCPP_INFO(get_logger(), "Sending goal %f,%f", goal.pose.pose.position.x, goal.pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleNavigateToPose::SharedPtr &goal_handle) {
            if (goal_handle) {
                isExploring = true;
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            } else {
                RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            }
        };

        send_goal_options.feedback_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &,
                const std::shared_ptr<const NavigateToPose::Feedback> &feedback) {
            RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
        };

        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result) {
            isExploring = false;
            clearFrontiers();
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code");
                    return;
            }
            RCLCPP_INFO(get_logger(), "Goal reached");
            rclcpp::shutdown();
            explore();
        };
        this->poseNavigator_->async_send_goal(goal, send_goal_options);

    }

    void saveMap(const string &mapName) {
        auto map_saver = std::make_shared<MapSaver>();
        map_saver->on_configure(rclcpp_lifecycle::State());
        SaveParameters save_parameters;
        save_parameters.map_file_name = mapName;
        map_saver->saveMapTopicToFile("map", save_parameters);
//        costmap_.saveMap(mapName);
    }

    std::optional<Point> findBoundary() {
        unsigned char *costmap_data = costmap_.getCharMap();
        const auto width = costmap_.getSizeInCellsX();
        const auto height = costmap_.getSizeInCellsY();

        for (unsigned int x = 0; x < width; ++x) {
            for (unsigned int y = 0; y < height; ++y) {
                unsigned int pos = costmap_.getIndex(x, y);
                const auto cost = costmap_data[pos];
                if (cost != FREE_SPACE) { continue; }
                if (checkNeighbors(costmap_data, width, height, x, y)) {
                    double ix, iy;
                    costmap_.mapToWorld(x, y, ix, iy);

                    Point boundary;
                    boundary.x = int(ix);
                    boundary.y = int(iy);
                    return boundary;
                }
            }
        }
        return std::nullopt;
    }

    bool
    checkNeighbors(const unsigned char *costmap_data, const unsigned int width, const unsigned int height,
                   unsigned int x,
                   unsigned int y) const {
        return (x + 1 < width &&
                costmap_data[costmap_.getIndex(x + 1, y)] == NO_INFORMATION) ||

               (x > 0 && costmap_data[costmap_.getIndex(x - 1, y)] == NO_INFORMATION) ||

               (y + 1 < height &&
                costmap_data[costmap_.getIndex(x, y + 1)] == NO_INFORMATION) ||

               (y > 0 &&
                costmap_data[costmap_.getIndex(x, y - 1)] == NO_INFORMATION) ||
               (x > 0 && y > 0 && costmap_data[costmap_.getIndex(x - 1, y - 1)] == NO_INFORMATION) ||
               (x + 1 < width && y + 1 < height &&
                costmap_data[costmap_.getIndex(x + 1, y + 1)] == NO_INFORMATION) ||
               (x + 1 < width && y > 0 &&
                costmap_data[costmap_.getIndex(x + 1, y - 1)] == NO_INFORMATION) ||
               (x > 0 && y + 1 < height &&
                costmap_data[costmap_.getIndex(x - 1, y + 1)] == NO_INFORMATION);
    }

};

int main(int argc, char *argv[]) {
    init(argc, argv);
    spin(make_shared<Mapper>());
    shutdown();
    return 0;
}