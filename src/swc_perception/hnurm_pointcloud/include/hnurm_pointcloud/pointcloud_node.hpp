#pragma once
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<tf2_ros/transform_listener.h>
#include<tf2_ros/buffer.h>
#include<sensor_msgs/msg/laser_scan.hpp>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
namespace hnurm {



class PointCloudNode : public rclcpp::Node{
  public:
    void run();
    PointCloudNode();
  private:
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pointcloud_to_laserscan(pcl::PointCloud<pcl::PointXYZI>& cloud);
  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    std::string base_frame_;
    std::string lidar_frame_;
  protected:
    std::shared_ptr<PointCloudNode> shared_from_this(){
      return std::static_pointer_cast<PointCloudNode>(rclcpp::Node::shared_from_this());
    }


};
}