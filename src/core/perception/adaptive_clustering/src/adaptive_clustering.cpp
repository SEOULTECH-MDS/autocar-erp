// Copyright (C) 2018  Zhi Yan and Li Sun

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "adaptive_clustering/msg/cluster_array.hpp"
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include "rclcpp_components/register_node_macro.hpp"

namespace perception
{
class AdaptiveClusteringNode : public rclcpp::Node
{
public:
  explicit AdaptiveClusteringNode(const rclcpp::NodeOptions & options)
  : Node("adaptive_clustering_node", options)
  {
    // Parameters
    this->declare_parameter<std::string>("sensor_model", "VLP-16");
    this->declare_parameter<bool>("print_fps", false);
    this->declare_parameter<float>("x_axis_min", -3.0);
    this->declare_parameter<float>("x_axis_max", 6.0);
    this->declare_parameter<float>("y_axis_min", -4.0);
    this->declare_parameter<float>("y_axis_max", 4.0);
    this->declare_parameter<float>("z_axis_min", -0.5);
    this->declare_parameter<float>("z_axis_max", 0.5);
    this->declare_parameter<int>("cluster_size_min", 3);
    this->declare_parameter<int>("cluster_size_max", 10000);
    this->declare_parameter<float>("tolerance_offset", 0.1);
    
    this->get_parameter("sensor_model", sensor_model_);
    this->get_parameter("print_fps", print_fps_);
    this->get_parameter("x_axis_min", x_axis_min_);
    this->get_parameter("x_axis_max", x_axis_max_);
    this->get_parameter("y_axis_min", y_axis_min_);
    this->get_parameter("y_axis_max", y_axis_max_);
    this->get_parameter("z_axis_min", z_axis_min_);
    this->get_parameter("z_axis_max", z_axis_max_);
    this->get_parameter("cluster_size_min", cluster_size_min_);
    this->get_parameter("cluster_size_max", cluster_size_max_);
    this->get_parameter("tolerance_offset", tolerance_offset_);

    if(sensor_model_.compare("VLP-16") == 0) {
      regions_[0] = 2; regions_[1] = 3; regions_[2] = 3; regions_[3] = 3; regions_[4] = 3;
      regions_[5] = 3; regions_[6] = 3; regions_[7] = 2; regions_[8] = 3; regions_[9] = 3;
      regions_[10]= 3; regions_[11]= 3; regions_[12]= 3; regions_[13]= 3;
    } else if (sensor_model_.compare("HDL-32E") == 0) {
      regions_[0] = 4; regions_[1] = 5; regions_[2] = 4; regions_[3] = 5; regions_[4] = 4;
      regions_[5] = 5; regions_[6] = 5; regions_[7] = 4; regions_[8] = 5; regions_[9] = 4;
      regions_[10]= 5; regions_[11]= 5; regions_[12]= 4; regions_[13]= 5;
    } else if (sensor_model_.compare("HDL-64E") == 0) {
      regions_[0] = 14; regions_[1] = 14; regions_[2] = 14; regions_[3] = 15; regions_[4] = 14;
    } else {
      RCLCPP_FATAL(this->get_logger(), "Unknown sensor model!");
    }

    // Publishers
    cluster_array_pub_ = this->create_publisher<adaptive_clustering::msg::ClusterArray>("clusters", 100);
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("poses", 100);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 100);
    
    // Subscriber
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 1, std::bind(&AdaptiveClusteringNode::pointCloudCallback, this, std::placeholders::_1));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ros_pc2_in) {
    if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);
    
    pcl::IndicesPtr pc_indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZ> pt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outskirt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> xfilter;
    pcl::PassThrough<pcl::PointXYZ> yfilter;

    xfilter.setInputCloud(pcl_pc_in);
    xfilter.setFilterFieldName("x");
    xfilter.setFilterLimits(-1.5, -0.2);
    xfilter.filter(*center);
    xfilter.setNegative(true);
    xfilter.filter(*outskirt);

    // 그 후 y축 방향으로 중앙에 있는 부분 제거
    yfilter.setInputCloud(center);
    yfilter.setFilterFieldName("y");
    yfilter.setFilterLimits(-0.5, 0.5);
    yfilter.setNegative(true);
    yfilter.filter(*pcl_pc_in);
    *pcl_pc_in += *outskirt;

    pt.setInputCloud(pcl_pc_in);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(z_axis_min_, z_axis_max_);
    pt.filter(*pcl_pc_in);
    
    pt.setInputCloud(pcl_pc_in);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(y_axis_min_, y_axis_max_);
    pt.filter(*pcl_pc_in);

    pt.setInputCloud(pcl_pc_in);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(x_axis_min_, x_axis_max_);
    pt.filter(*pc_indices);

    std::array<std::vector<int>, region_max_> indices_array;
    for(long i = 0; i < pc_indices->size(); i++) {
      float range = 0.0;
      for(int j = 0; j < region_max_; j++) {
        float d2 = pcl_pc_in->points[(*pc_indices)[i]].x * pcl_pc_in->points[(*pc_indices)[i]].x +
      pcl_pc_in->points[(*pc_indices)[i]].y * pcl_pc_in->points[(*pc_indices)[i]].y +
      pcl_pc_in->points[(*pc_indices)[i]].z * pcl_pc_in->points[(*pc_indices)[i]].z;
        if(d2 > range * range && d2 <= (range+regions_[j]) * (range+regions_[j])) {
          indices_array[j].push_back((*pc_indices)[i]);
          break;
        }
        range += regions_[j];
      }
    }
    
    float tolerance = 0.0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr > > clusters;
    
    for(int i = 0; i < region_max_; i++) {
      tolerance += tolerance_offset_;
      if(indices_array[i].size() > (unsigned int)cluster_size_min_) {
        std::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pcl_pc_in, indices_array_ptr);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(cluster_size_min_);
        ec.setMaxClusterSize(cluster_size_max_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcl_pc_in);
        ec.setIndices(indices_array_ptr);
        ec.extract(cluster_indices);
        
        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
          for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster->points.push_back(pcl_pc_in->points[*pit]);
        }
          cluster->width = cluster->size();
          cluster->height = 1;
          cluster->is_dense = true;
      clusters.push_back(cluster);
        }
      }
    }
    
    adaptive_clustering::msg::ClusterArray cluster_array;
    geometry_msgs::msg::PoseArray pose_array;
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::MarkerArray marker_pose;

    for(long i = 0; i < clusters.size(); i++) 
    {
      if(cluster_array_pub_->get_subscription_count() > 0) 
      {
        sensor_msgs::msg::PointCloud2 ros_pc2_out;
        pcl::toROSMsg(*clusters[i], ros_pc2_out);
        cluster_array.clusters.push_back(ros_pc2_out);
      }
      
      if(marker_array_pub_->get_subscription_count() > 0) 
      {
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(*clusters[i], min, max);
        
        visualization_msgs::msg::Marker marker;
        
        marker.header = ros_pc2_in->header;
        marker.ns = "adaptive_clustering";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        
        geometry_msgs::msg::Point p[24];
        p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
        p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
        p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
        p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
        p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
        p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
        p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
        p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
        p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
        p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
        p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
        p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
        p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
        p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
        p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
        p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
        p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
        p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
        p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
        p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
        p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
        p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
        p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
        p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
        for(int k = 0; k < 24; k++) {
      marker.points.push_back(p[k]);
        }
        
        size_x_ = std::abs(max[0]-min[0]);
        size_y_ = std::abs(max[1]-min[1]);
        size_z_ = std::abs(max[2]-min[2]);
        
        marker.scale.x = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.5;

        marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        if (size_z_ > 0.2 && size_x_ <1 && size_y_ <1 && size_z_ <1.5) 
        {
          marker_array.markers.push_back(marker);
        }
        
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "clustering_pose";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;

        marker.points.clear();
        marker.scale.x = std::abs(max[0]-min[0]);
        marker.scale.y = std::abs(max[1]-min[1]);
        marker.scale.z = std::abs(max[2]-min[2]);
        marker.pose.position.x = (max[0]+min[0])/2;
        marker.pose.position.y = (max[1]+min[1])/2;
        marker.pose.position.z = (max[2]+min[2])/2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.5;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        if ( size_z_ > 0.2 && size_x_ <1 && size_y_ <1 && size_z_ <1.5 )
        {
          marker_pose.markers.push_back(marker);
          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*clusters[i], centroid);
          
          geometry_msgs::msg::Pose pose;
          pose.position.x = centroid[0];
          pose.position.y = centroid[1];
          pose.position.z = centroid[2];
          pose.orientation.x = std::abs(max[0]-min[0]);
          pose.orientation.y = std::abs(max[1]-min[1]);
          pose.orientation.z = std::abs(max[2]-min[2]);
          pose.orientation.w = 1;
          pose_array.poses.push_back(pose);
        }
      }
    }
    
    if(cluster_array.clusters.size()) {
      cluster_array.header = ros_pc2_in->header;
      cluster_array_pub_->publish(cluster_array);
    }

    if(pose_array.poses.size()) {
      pose_array.header = ros_pc2_in->header;
      pose_array_pub_->publish(pose_array);
    }
    
    if(marker_array.markers.size()) {
      marker_array_pub_->publish(marker_array);
    }
    
    if(print_fps_)if(++frames>10){RCLCPP_INFO(this->get_logger(), "[adaptive_clustering] fps = %f, timestamp = %ld", float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC), clock()/CLOCKS_PER_SEC);reset = true;}//fps
  }

  rclcpp::Publisher<adaptive_clustering::msg::ClusterArray>::SharedPtr cluster_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  bool print_fps_;
  float x_axis_min_;
  float x_axis_max_;
  float y_axis_min_;
  float y_axis_max_;
  float z_axis_min_;
  float z_axis_max_;
  float tolerance_offset_;
  int cluster_size_min_;
  int cluster_size_max_;
  float size_x_;
  float size_y_;
  float size_z_;

  static const int region_max_ = 10;
  int regions_[13];
  std::string sensor_model_;

  int frames; 
  clock_t start_time; 
  bool reset = true;
};

} // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::AdaptiveClusteringNode)
