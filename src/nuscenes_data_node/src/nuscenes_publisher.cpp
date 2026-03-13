#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <fstream>
#include <vector>

class NuscenesPublisher : public rclcpp::Node {
public:
    NuscenesPublisher() : Node("nuscenes_publisher") {
        // 두 개의 분리된 토픽으로 발행합니다.
        ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10);
        obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_points", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NuscenesPublisher::publish_data, this));
    }

private:
    void publish_data() {
        std::string bin_file_path = "/workspace/data/nuScenes-mini/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // 1. 데이터 읽기 (이전과 동일)
        std::ifstream file(bin_file_path, std::ios::binary);
        if (file.is_open()) {
            file.seekg(0, std::ios::end);
            size_t num_points = file.tellg() / (5 * sizeof(float)); 
            file.seekg(0, std::ios::beg);
            cloud->points.resize(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                float x, y, z, intensity, ring_index;
                file.read(reinterpret_cast<char*>(&x), sizeof(float));
                file.read(reinterpret_cast<char*>(&y), sizeof(float));
                file.read(reinterpret_cast<char*>(&z), sizeof(float));
                file.read(reinterpret_cast<char*>(&intensity), sizeof(float));
                file.read(reinterpret_cast<char*>(&ring_index), sizeof(float));
                cloud->points[i].x = x; cloud->points[i].y = y; cloud->points[i].z = z; cloud->points[i].intensity = intensity;
            }
            file.close();
        } else {
            return;
        }

        // 2. RANSAC Segmentation 적용
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE); // 평면 모델 추출
        seg.setMethodType(pcl::SAC_RANSAC);    // RANSAC 알고리즘 사용
        seg.setMaxIterations(100);             // 최대 반복 횟수
        seg.setDistanceThreshold(0.2);         // 오차 허용 범위 (20cm 이내의 점을 바닥으로 간주)
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
            return;
        }

        // 3. 인덱스를 바탕으로 바닥(Ground)과 장애물(Obstacle) 분리
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false); // Inlier(바닥) 추출
        extract.filter(*ground_cloud);

        extract.setNegative(true);  // Outlier(장애물) 추출
        extract.filter(*obstacle_cloud);

        // 4. ROS2 메시지 변환 및 퍼블리시
        sensor_msgs::msg::PointCloud2 ground_msg, obstacle_msg;
        pcl::toROSMsg(*ground_cloud, ground_msg);
        pcl::toROSMsg(*obstacle_cloud, obstacle_msg);

        auto now = this->get_clock()->now();
        ground_msg.header.frame_id = "nuscenes_lidar_frame"; ground_msg.header.stamp = now;
        obstacle_msg.header.frame_id = "nuscenes_lidar_frame"; obstacle_msg.header.stamp = now;

        ground_pub_->publish(ground_msg);
        obstacle_pub_->publish(obstacle_msg);

        RCLCPP_INFO(this->get_logger(), "Ground: %zu pts, Obstacles: %zu pts", ground_cloud->points.size(), obstacle_cloud->points.size());
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuscenesPublisher>());
    rclcpp::shutdown();
    return 0;
}