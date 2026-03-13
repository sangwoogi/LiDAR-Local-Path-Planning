#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>

class GridMapNode : public rclcpp::Node {
public:
    GridMapNode() : Node("grid_map_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "obstacle_points", 10, std::bind(&GridMapNode::pointcloud_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
        
        resolution_ = 0.2; 
        width_ = 50.0;     
        height_ = 50.0;    
        
        grid_size_x_ = std::ceil(width_ / resolution_);
        grid_size_y_ = std::ceil(height_ / resolution_);
        
        origin_x_ = -width_ / 2.0;
        origin_y_ = -height_ / 2.0;

        // --- C-Space 파라미터 ---
        robot_radius_ = 1.0; // 로봇의 물리적 반지름 (1.0m로 설정하여 넉넉한 안전거리 확보)
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header = msg->header; 
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_size_x_;
        grid_msg.info.height = grid_size_y_;
        grid_msg.info.origin.position.x = origin_x_;
        grid_msg.info.origin.position.y = origin_y_;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        // 1. 원본 장애물을 기록할 임시 Base Grid (0으로 초기화)
        std::vector<int8_t> base_grid(grid_size_x_ * grid_size_y_, 0);

        // 2. 3D 포인트를 2D 격자에 투영 (원본 장애물 위치 기록)
        for (const auto& point : cloud->points) {
            if (point.z < -1.0 || point.z > 2.0) continue;

            int i = std::floor((point.x - origin_x_) / resolution_);
            int j = std::floor((point.y - origin_y_) / resolution_);

            if (i >= 0 && i < grid_size_x_ && j >= 0 && j < grid_size_y_) {
                base_grid[j * grid_size_x_ + i] = 100;
            }
        }

        // 최종 퍼블리시할 Grid 역시 0으로 초기화
        grid_msg.data.assign(grid_size_x_ * grid_size_y_, 0);

        // 3. C-Space Dilation (팽창 연산)
        // 로봇 반지름을 격자 칸 수로 변환
        int r_cells = std::ceil(robot_radius_ / resolution_);

        for (int y = 0; y < grid_size_y_; ++y) {
            for (int x = 0; x < grid_size_x_; ++x) {
                // 원본 지도에 장애물이 있는 경우
                if (base_grid[y * grid_size_x_ + x] == 100) {
                    
                    // 주변 r_cells 반경 안의 격자들을 모두 장애물(100)로 칠함 (원형 마스크 씌우기)
                    for (int dy = -r_cells; dy <= r_cells; ++dy) {
                        for (int dx = -r_cells; dx <= r_cells; ++dx) {
                            // 유클리디안 거리 공식을 통해 원형 반경 내의 격자만 선택
                            if (dx * dx + dy * dy <= r_cells * r_cells) {
                                int nx = x + dx;
                                int ny = y + dy;
                                
                                // 맵 경계선 안쪽인지 확인
                                if (nx >= 0 && nx < grid_size_x_ && ny >= 0 && ny < grid_size_y_) {
                                    grid_msg.data[ny * grid_size_x_ + nx] = 100;
                                }
                            }
                        }
                    }
                }
            }
        }

        // --- 내 차 주변(원점) 강제 클리어 ---
        // 차량 중심(0,0) 기준 1.5m 반경 안의 장애물을 모두 0(자유 공간)으로 지워버립니다.
        int ego_clear_cells = std::ceil(1.5 / resolution_); 
        int center_grid_x = std::floor((0.0 - origin_x_) / resolution_);
        int center_grid_y = std::floor((0.0 - origin_y_) / resolution_);

        for (int dy = -ego_clear_cells; dy <= ego_clear_cells; ++dy) {
            for (int dx = -ego_clear_cells; dx <= ego_clear_cells; ++dx) {
                if (dx * dx + dy * dy <= ego_clear_cells * ego_clear_cells) {
                    int nx = center_grid_x + dx;
                    int ny = center_grid_y + dy;
                    if (nx >= 0 && nx < grid_size_x_ && ny >= 0 && ny < grid_size_y_) {
                        grid_msg.data[ny * grid_size_x_ + nx] = 0; // 강제로 길 터주기
                    }
                }
            }
        }
        
        publisher_->publish(grid_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    
    double resolution_, width_, height_, origin_x_, origin_y_, robot_radius_;
    int grid_size_x_, grid_size_y_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridMapNode>());
    rclcpp::shutdown();
    return 0;
}