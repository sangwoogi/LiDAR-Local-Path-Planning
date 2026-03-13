#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

struct AStarNode {
    int x, y;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<AStarNode> parent;

    AStarNode(int _x, int _y, double _g, double _h, std::shared_ptr<AStarNode> _p = nullptr)
        : x(_x), y(_y), g_cost(_g), h_cost(_h), f_cost(_g + _h), parent(_p) {}

    bool operator>(const AStarNode& other) const { return f_cost > other.f_cost; }
};

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner_node") {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "occupancy_grid", 10, std::bind(&PathPlannerNode::map_callback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        
        has_map_ = false;
        RCLCPP_INFO(this->get_logger(), "Path Planner Ready. Waiting for goal...");
    }

private:
    nav_msgs::msg::OccupancyGrid current_map_;
    bool has_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        has_map_ = true;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!has_map_) return;
        RCLCPP_INFO(this->get_logger(), "Goal received! Planning path...");

        double start_world_x = 0.0, start_world_y = 0.0;
        double goal_world_x = msg->pose.position.x, goal_world_y = msg->pose.position.y;

        int start_grid_x, start_grid_y, goal_grid_x, goal_grid_y;
        world_to_grid(start_world_x, start_world_y, start_grid_x, start_grid_y);
        world_to_grid(goal_world_x, goal_world_y, goal_grid_x, goal_grid_y);

        auto grid_path = plan_astar(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y);

        if (!grid_path.empty()) {
            // 1. 격자 인덱스를 실제 물리 좌표(m)로 변환
            std::vector<std::pair<double, double>> world_path;
            for (const auto& p : grid_path) {
                double wx = p.first * current_map_.info.resolution + current_map_.info.origin.position.x;
                double wy = p.second * current_map_.info.resolution + current_map_.info.origin.position.y;
                world_path.push_back({wx, wy});
            }

            // 2. Cubic Spline 평활화 적용
            auto smoothed_path = smooth_path_catmull_rom(world_path);
            
            // 3. 퍼블리시
            publish_path(smoothed_path);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to find a valid path.");
        }
    }

    // --- Catmull-Rom 스플라인 평활화 함수 ---
    std::vector<std::pair<double, double>> smooth_path_catmull_rom(const std::vector<std::pair<double, double>>& path) {
        if (path.size() < 4) return path; // 점이 너무 적으면 그대로 반환

        // A*의 모든 점을 다 쓰면 너무 촘촘하므로, 일정한 간격(예: 3칸)으로 제어점(Control Points) 추출
        std::vector<std::pair<double, double>> control_points;
        for (size_t i = 0; i < path.size(); i += 3) {
            control_points.push_back(path[i]);
        }
        if (control_points.back() != path.back()) control_points.push_back(path.back()); // 마지막 목적지 강제 포함

        // Catmull-Rom 연산을 위해 양 끝에 가상의 점 추가 (경계 조건 처리)
        control_points.insert(control_points.begin(), control_points.front());
        control_points.push_back(control_points.back());

        std::vector<std::pair<double, double>> smoothed_path;
        
        // 4개의 점을 한 세트로 묶어서 3차 곡선 보간
        for (size_t i = 1; i < control_points.size() - 2; ++i) {
            auto p0 = control_points[i - 1];
            auto p1 = control_points[i];
            auto p2 = control_points[i + 1];
            auto p3 = control_points[i + 2];

            // t를 0부터 1까지 잘게 쪼개어 곡선 위의 점들을 생성 (10단계 보간)
            for (double t = 0.0; t < 1.0; t += 0.1) {
                double t2 = t * t;
                double t3 = t2 * t;

                double x = 0.5 * ((2 * p1.first) +
                                  (-p0.first + p2.first) * t +
                                  (2 * p0.first - 5 * p1.first + 4 * p2.first - p3.first) * t2 +
                                  (-p0.first + 3 * p1.first - 3 * p2.first + p3.first) * t3);
                                  
                double y = 0.5 * ((2 * p1.second) +
                                  (-p0.second + p2.second) * t +
                                  (2 * p0.second - 5 * p1.second + 4 * p2.second - p3.second) * t2 +
                                  (-p0.second + 3 * p1.second - 3 * p2.second + p3.second) * t3);

                smoothed_path.push_back({x, y});
            }
        }
        // 정확한 목적지 도달을 위해 마지막 점 추가
        smoothed_path.push_back(path.back());

        return smoothed_path;
    }

    // --- 기존 A* 알고리즘 ---
    std::vector<std::pair<int, int>> plan_astar(int start_x, int start_y, int goal_x, int goal_y) {
        int width = current_map_.info.width;
        int height = current_map_.info.height;

        if (goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) return {};
        if (current_map_.data[goal_y * width + goal_x] == 100) return {};

        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        std::vector<std::vector<bool>> closed_list(width, std::vector<bool>(height, false));

        open_list.emplace(start_x, start_y, 0.0, calculate_heuristic(start_x, start_y, goal_x, goal_y));

        int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

        while (!open_list.empty()) {
            AStarNode current = open_list.top();
            open_list.pop();

            if (current.x == goal_x && current.y == goal_y) {
                std::vector<std::pair<int, int>> path;
                std::shared_ptr<AStarNode> curr_ptr = std::make_shared<AStarNode>(current);
                while (curr_ptr != nullptr) {
                    path.push_back({curr_ptr->x, curr_ptr->y});
                    curr_ptr = curr_ptr->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            if (closed_list[current.x][current.y]) continue;
            closed_list[current.x][current.y] = true;

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    if (current_map_.data[ny * width + nx] != 100 && !closed_list[nx][ny]) {
                        double move_cost = (i < 4) ? 1.0 : 1.414; 
                        double new_g = current.g_cost + move_cost;
                        double h = calculate_heuristic(nx, ny, goal_x, goal_y);
                        
                        open_list.emplace(nx, ny, new_g, h, std::make_shared<AStarNode>(current));
                    }
                }
            }
        }
        return {}; 
    }

    double calculate_heuristic(int x1, int y1, int x2, int y2) {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

    void world_to_grid(double wx, double wy, int& gx, int& gy) {
        gx = std::floor((wx - current_map_.info.origin.position.x) / current_map_.info.resolution);
        gy = std::floor((wy - current_map_.info.origin.position.y) / current_map_.info.resolution);
    }

    void publish_path(const std::vector<std::pair<double, double>>& smoothed_path) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = current_map_.header.frame_id;
        path_msg.header.stamp = this->get_clock()->now();

        for (const auto& p : smoothed_path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.first;
            pose.pose.position.y = p.second;
            pose.pose.position.z = 0.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Smoothed Path published with %zu waypoints.", path_msg.poses.size());
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}