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
#include "angles/angles.h"
#include "std_msgs/std_msgs/msg/color_rgba.hpp"

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
using angles::to_degrees;
using angles::from_degrees;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;
using namespace rclcpp;
using namespace rclcpp_action;
using std::chrono::steady_clock;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Mapper : public Node {
public:
    Mapper()
            : Node("mapper") {
        mapSubscription_ = this->create_subscription<OccupancyGrid>(
                "/map", 10, bind(&Mapper::updateFullMap, this, _1));

        mapUpdatesSubscription_ = this->create_subscription<OccupancyGridUpdate>(
                "/map_updates", 10, bind(&Mapper::updatePartialMap, this, _1));

//        controlLoopTimer_ = this->create_wall_timer(
//                CONTROL_LOOP_INTERVAL_MILLI_SEC, [this] { controlLoop(); });
        this->poseNavigator_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose");
        sleep(10);//Wait till bt_navigator is active //TODO

        poseNavigator_->wait_for_action_server();
    }

private:
    Costmap2D costmap_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr poseNavigator_;

    Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    Subscription<OccupancyGridUpdate>::SharedPtr mapUpdatesSubscription_;
    bool isExploring;

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
        if (!isExploring) {
            isExploring = true;
            explore();
        }
    }

    void updatePartialMap(OccupancyGridUpdate::UniquePtr msg) {
        RCLCPP_DEBUG(get_logger(), "received partial map update");
        if (msg->x < 0 || msg->y < 0) {
            RCLCPP_ERROR(get_logger(), "negative coordinates, invalid update. x: %d, y: %d", msg->x,
                         msg->y);
            return;
        }

        size_t x0 = static_cast<size_t>(msg->x);
        size_t y0 = static_cast<size_t>(msg->y);
        size_t xn = msg->width + x0;
        size_t yn = msg->height + y0;

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        lock_guard<Costmap2D::mutex_t> lock(*mutex);

        size_t costmap_xn = costmap_.getSizeInCellsX();
        size_t costmap_yn = costmap_.getSizeInCellsY();

        if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
            y0 > costmap_yn) {
            RCLCPP_WARN(get_logger(), "received update doesn't fully fit into existing map, "
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

    void stop() {
        RCLCPP_INFO(get_logger(), "Stopped...");
//        controlLoopTimer_->cancel();
        poseNavigator_->async_cancel_all_goals();
    }

    void explore() {
        auto target_position = findBoundary();
        auto goal = NavigateToPose::Goal();
        target_position.x = -8;
        target_position.y = 1;
        goal.pose.pose.position = target_position;
//        goal.pose.pose.orientation.w = 1.;
        goal.pose.header.frame_id = "map";

        RCLCPP_INFO(get_logger(), "Sending goal %f,%f", target_position.x, target_position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleNavigateToPose::SharedPtr &goal_handle) {
            if (goal_handle) {
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
//            explore();
        };
        this->poseNavigator_->async_send_goal(goal, send_goal_options);

    }

    void saveMap(const string &mapName) {
        costmap_.saveMap(mapName);
    }

    Point findBoundary() {
        Point p;
        unsigned char *costmap_data = costmap_.getCharMap();
        const auto width = costmap_.getSizeInCellsX();
        const auto height = costmap_.getSizeInCellsY();

        for (unsigned int x = 0; x < width; ++x) {
//            string row = "";
            for (unsigned int y = 0; y < height; ++y) {
                unsigned int pos = costmap_.getIndex(x, y);
                const auto cost = costmap_data[pos];
                if (cost == FREE_SPACE) {
                    if ((x + 1 < width &&
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
                         costmap_data[costmap_.getIndex(x - 1, y + 1)] == NO_INFORMATION)) {


                        double ix, iy;
                        costmap_.mapToWorld(x, y, ix, iy);

                        p.x = ix;
                        p.y = iy;
                        return p;
                    }
//                    RCLCPP_INFO(get_logger(), row.c_str());
                }
            }
        }
        RCLCPP_ERROR(get_logger(), "NO BOUNDARIES FOUND!!");
        return p;
    }

};

int main(int argc, char *argv[]) {
    init(argc, argv);
    spin(make_shared<Mapper>());
    shutdown();
    return 0;
}