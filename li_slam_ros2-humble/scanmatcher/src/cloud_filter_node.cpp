#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

class CloudFilterNode : public rclcpp::Node
{
public:
    CloudFilterNode() : Node("cloud_filter_node")
    {
        // 파라미터 선언
        this->declare_parameter("min_range", 0.1);
        this->declare_parameter("max_range", 100.0);
        this->declare_parameter("remove_nan", true);
        this->declare_parameter("voxel_size", 0.1);

        // 파라미터 가져오기
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        remove_nan_ = this->get_parameter("remove_nan").as_bool();
        voxel_size_ = this->get_parameter("voxel_size").as_double();

        // Publisher와 Subscriber 설정
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input", 10, std::bind(&CloudFilterNode::cloudCallback, this, std::placeholders::_1));
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // ROS 메시지를 PCL 포인트 클라우드로 변환
        pcl::fromROSMsg(*msg, *cloud);

        // NaN 포인트 제거
        if (remove_nan_) {
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        }

        // Voxel grid 필터 적용
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        vg.filter(*cloud_filtered);

        // 거리 기반 필터링
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_range(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud_filtered->points) {
            float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (range >= min_range_ && range <= max_range_) {
                cloud_range->points.push_back(point);
            }
        }

        // PCL 포인트 클라우드를 ROS 메시지로 변환
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_range, output);
        output.header = msg->header;

        // 발행
        pub_->publish(output);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    double min_range_, max_range_, voxel_size_;
    bool remove_nan_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}