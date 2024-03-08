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


        poseSubscription_ = this->create_subscription<PoseWithCovarianceStamped>(
                "/pose", 10, bind(&Mapper::poseTopicCallback, this, _1));

        marker_array_publisher_ = this->create_publisher<MarkerArray>("/frontiers", 10);

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
    struct Frontier {
        uint32_t size;
        double min_distance;
        double cost;
        Point initial;
        Point centroid;
        Point middle;
        vector<Point> points;
    };
    rclcpp_action::Client<NavigateToPose>::SharedPtr poseNavigator_;
//    TimerBase::SharedPtr controlLoopTimer_;
//    static constexpr milliseconds CONTROL_LOOP_INTERVAL_MILLI_SEC = 500ms;

    Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    Subscription<OccupancyGridUpdate>::SharedPtr mapUpdatesSubscription_;
    Publisher<MarkerArray>::SharedPtr marker_array_publisher_;

    Subscription<PoseWithCovarianceStamped>::SharedPtr poseSubscription_;
    PoseWithCovarianceStamped::UniquePtr pose_;


    unsigned char *map_;
    unsigned int size_x_, size_y_;
    double potential_scale_ = 1e-3, gain_scale_ = 1.0;
    double min_frontier_size_ = 0.5;
    size_t last_markers_count_ = 0;
    vector<Point> frontier_blacklist_;
    Point currentPosition;
    const std::string global_frame_ = "map";      ///< @brief The global frame for the costmap
    const std::string robot_base_frame_ = "base_link";  ///< @brief The frame_id of the robot base
//    const tf::TransformListener *const tf_;//TODO

    array<unsigned char, 256> init_translation_table() {
        array<unsigned char, 256> cost_translation_table;

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
            unsigned char cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
            costmap_data[i] = cost_translation_table_[cell_cost];
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

    void poseTopicCallback(PoseWithCovarianceStamped::UniquePtr pose) {
        if (pose_ == nullptr) {
            pose_ = move(pose);
            currentPosition = pose_->pose.pose.position;
            RCLCPP_INFO(get_logger(), "********************* START EXPLORING *********************");
            explore();
        }
    }

    bool goalOnBlacklist(const Point &goal) {
        constexpr static size_t tolerance = 5;
        // check if a goal is on the blacklist for goals that we're pursuing
        for (auto &frontier_goal: frontier_blacklist_) {
            double x_diff = fabs(goal.x - frontier_goal.x);
            double y_diff = fabs(goal.y - frontier_goal.y);

            if (x_diff < tolerance * costmap_.getResolution() &&
                y_diff < tolerance * costmap_.getResolution())
                return true;
        }
        return false;
    }

    void visualizeFrontiers(const std::vector<Frontier> &frontiers) {
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

        m.header.frame_id = pose_->header.frame_id;
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
//        m.lifetime = rclcpp::Duration(0);
        m.frame_locked = true;

        // weighted frontiers are always sorted
        double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

        m.action = Marker::ADD;
        size_t id = 0;
        for (auto &frontier: frontiers) {
            RCLCPP_INFO(get_logger(), "visualising %f,%f ", frontier.centroid.x, frontier.centroid.y);
            m.type = Marker::POINTS;
            m.id = int(id);
//            m.pose.position = {};
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;
            m.points = frontier.points;
            if (goalOnBlacklist(frontier.centroid)) {
                m.color = red;
            } else {
                m.color = blue;
            }
            markers.push_back(m);
            ++id;
            m.type = Marker::SPHERE;
            m.id = int(id);
            m.pose.position = frontier.initial;
            // scale frontier according to its cost (costier frontiers will be smaller)
            double scale = min(abs(min_cost * 0.4 / frontier.cost), 0.5);
            m.scale.x = scale;
            m.scale.y = scale;
            m.scale.z = scale;
            m.points = {};
            m.color = green;
            markers.push_back(m);
            ++id;
        }
        size_t current_markers_count = markers.size();

        // delete previous markers, which are now unused
        m.action = Marker::DELETE;
        for (; id < last_markers_count_; ++id) {
            m.id = int(id);
            markers.push_back(m);
        }

        last_markers_count_ = current_markers_count;
        marker_array_publisher_->publish(markers_msg);
    }

    void stop() {
        RCLCPP_INFO(get_logger(), "Stopped...");
//        controlLoopTimer_->cancel();
        poseNavigator_->async_cancel_all_goals();
    }

    void explore() {
        auto frontiers = searchFrom(currentPosition);
        if (frontiers.empty()) {
            RCLCPP_INFO(get_logger(), "Empty frontiers...");
            stop();
            return;
        }
        // find non blacklisted frontier
        auto frontier =
                std::find_if_not(frontiers.begin(), frontiers.end(),
                                 [this](const Frontier &f) {
                                     return goalOnBlacklist(f.centroid);
                                 });
        if (frontier == frontiers.end()) {
            stop();
            return;
        }
        std::vector<Frontier> others;
        others.push_back(*frontier);
        visualizeFrontiers(others);
        Point target_position = frontier->centroid;
        frontier_blacklist_.push_back(target_position);
        auto goal = NavigateToPose::Goal();
        target_position.x = -5.62079;
        target_position.y = -4.40925;
        goal.pose.pose.position = target_position;
//        goal.pose.pose.orientation.w = 1.;
        goal.pose.header.frame_id = global_frame_;

        RCLCPP_INFO(get_logger(), "Sending goal %f,%f", target_position.x, target_position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleNavigateToPose::SharedPtr &goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        send_goal_options.feedback_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &,
                const std::shared_ptr<const NavigateToPose::Feedback> &feedback) {
            currentPosition = feedback->current_pose.pose.position;
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
            explore();
        };
        this->poseNavigator_->async_send_goal(goal, send_goal_options);

    }

    void saveMap(const string &mapName) {
        costmap_.saveMap(mapName);
    }

    vector<unsigned int> nhood4(unsigned int idx,
                                const Costmap2D &costmap) {
        // get 4-connected neighbourhood indexes, check for edge of map
        vector<unsigned int> out;

        unsigned int size_x_ = costmap.getSizeInCellsX(),
                size_y_ = costmap.getSizeInCellsY();

        if (idx > size_x_ * size_y_ - 1) {
            RCLCPP_WARN(get_logger(), "Evaluating nhood for offmap point");
            return out;
        }

        if (idx % size_x_ > 0) {
            out.push_back(idx - 1);
        }
        if (idx % size_x_ < size_x_ - 1) {
            out.push_back(idx + 1);
        }
        if (idx >= size_x_) {
            out.push_back(idx - size_x_);
        }
        if (idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx + size_x_);
        }
        return out;
    }

    Frontier buildNewFrontier(unsigned int initial_cell,
                              unsigned int reference,
                              vector<bool> &frontier_flag) {
        // initialize frontier structure
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;
        output.size = 1;
        output.min_distance = numeric_limits<double>::infinity();

        // record initial contact point for frontier
        unsigned int ix, iy;
        costmap_.indexToCells(initial_cell, ix, iy);
        costmap_.mapToWorld(ix, iy, output.initial.x, output.initial.y);

        // push initial gridcell onto queue
        queue<unsigned int> bfs;
        bfs.push(initial_cell);

        // cache reference position in world coords
        unsigned int rx, ry;
        double reference_x, reference_y;
        costmap_.indexToCells(reference, rx, ry);
        costmap_.mapToWorld(rx, ry, reference_x, reference_y);

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int nbr: nhood8(idx, costmap_)) {
                // check if neighbour is a potential frontier cell
                if (isNewFrontierCell(nbr, frontier_flag)) {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_.indexToCells(nbr, mx, my);
                    costmap_.mapToWorld(mx, my, wx, wy);

                    Point point;
                    point.x = wx;
                    point.y = wy;
                    output.points.push_back(point);

                    // update frontier size
                    output.size++;

                    // update centroid of frontier
                    output.centroid.x += wx;
                    output.centroid.y += wy;

                    // determine frontier's distance from robot, going by closest gridcell
                    // to robot
                    double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                                           pow((double(reference_y) - double(wy)), 2.0));
                    if (distance < output.min_distance) {
                        output.min_distance = distance;
                        output.middle.x = wx;
                        output.middle.y = wy;
                    }

                    // add to queue for breadth first search
                    bfs.push(nbr);
                }
            }
        }

        // average out frontier centroid
        output.centroid.x /= output.size;
        output.centroid.y /= output.size;
        return output;
    }

    double frontierCost(const Frontier &frontier) {
        return (potential_scale_ * frontier.min_distance *
                costmap_.getResolution()) -
               (gain_scale_ * frontier.size * costmap_.getResolution());
    }

    bool isNewFrontierCell(unsigned int idx,
                           const vector<bool> &frontier_flag) {
        // check that cell is unknown and not already marked as frontier
        if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
            return false;
        }

        // frontier cells should have at least one cell in 4-connected neighbourhood
        // that is free
        for (unsigned int nbr: nhood4(idx, costmap_)) {
            if (map_[nbr] == FREE_SPACE) {
                return true;
            }
        }

        return false;
    }

    vector<Frontier> searchFrom(Point position) {
        vector<Frontier> frontier_list;

        // Sanity check that robot is inside costmap bounds before searching
        unsigned int mx, my;
        if (!costmap_.worldToMap(position.x, position.y, mx, my)) {
            RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

        // make sure map is consistent and locked for duration of search
        lock_guard<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        map_ = costmap_.getCharMap();
        size_x_ = costmap_.getSizeInCellsX();
        size_y_ = costmap_.getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        vector<bool> frontier_flag(size_x_ * size_y_,
                                   false);
        vector<bool> visited_flag(size_x_ * size_y_,
                                  false);

        // initialize breadth first search
        queue<unsigned int> bfs;

        unsigned int pos = costmap_.getIndex(mx, my);
        bfs.push(pos);
        visited_flag[bfs.front()] = true;

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // iterate over 4-connected neighbourhood
            for (unsigned nbr: nhood4(idx, costmap_)) {
                // add to queue all free, unvisited cells, use descending search in case
                // initialized on non-free cell
                if (map_[nbr] == FREE_SPACE && !visited_flag[nbr]) {
                    visited_flag[nbr] = true;
                    bfs.push(nbr);
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
                    // neighbour)
                } else if (map_[nbr] == NO_INFORMATION) {
                    frontier_flag[idx] = true;
                    Frontier new_frontier = buildNewFrontier(idx, pos, frontier_flag);
                    if (new_frontier.size * costmap_.getResolution() >=
                        min_frontier_size_) {
                        frontier_list.push_back(new_frontier);
                    }
                }
            }
        }

        // set costs of frontiers
        for (auto &frontier: frontier_list) {
            frontier.cost = frontierCost(frontier);
        }
        sort(
                frontier_list.begin(), frontier_list.end(),
                [](const Frontier &f1, const Frontier &f2) { return f1.cost < f2.cost; });

        return frontier_list;
    }

    vector<unsigned int> nhood8(unsigned int idx,
                                const Costmap2D &costmap) {
        // get 8-connected neighbourhood indexes, check for edge of map
        vector<unsigned int> out = nhood4(idx, costmap);

        unsigned int size_x_ = costmap.getSizeInCellsX(),
                size_y_ = costmap.getSizeInCellsY();

        if (idx > size_x_ * size_y_ - 1) {
            return out;
        }

        if (idx % size_x_ > 0 && idx >= size_x_) {
            out.push_back(idx - 1 - size_x_);
        }
        if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx - 1 + size_x_);
        }
        if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
            out.push_back(idx + 1 - size_x_);
        }
        if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx + 1 + size_x_);
        }

        return out;
    }

};

int main(int argc, char *argv[]) {
    init(argc, argv);
    spin(make_shared<Mapper>());
    shutdown();
    return 0;
}