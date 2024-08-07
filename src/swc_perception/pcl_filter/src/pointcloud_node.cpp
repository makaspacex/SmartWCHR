#include "pcl_filter/pointcloud_node.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

namespace hnurm
{

  PointCloudNode::PointCloudNode() : Node("pcl_filter_node")
  {
    RCLCPP_INFO(get_logger(), "Pointcloud node created");
    declare_parameter("lidar_topic", "/livox/lidar/pointcloud"); // segmentation/obstacle    /livox/lidar/pointcloud
    declare_parameter("pointcloud_output_topic", "/pcl_filter");
  }

  void PointCloudNode::run()
  {
    RCLCPP_INFO(get_logger(), "Running pointcloud node");
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("lidar_topic").as_string(),
      1,
      std::bind(&PointCloudNode::pointcloud_callback, shared_from_this(), 
      std::placeholders::_1)
      );
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(get_parameter("pointcloud_output_topic").as_string(),1);
  }

  void PointCloudNode::pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    // 将接收到的点云消息转换为PCL点云格式
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 使用Passthrough滤波器对点云进行裁剪
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.6);
    pass.filter(cloud);

    // 使用ConditionalRemoval滤波器对点云进行裁剪
    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    pcl::ConditionOr<pcl::PointXYZI>::Ptr condition(new pcl::ConditionOr<pcl::PointXYZI>());
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 0.2)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, -1)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, 0.15)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, -0.8)));

    condrem.setCondition(condition);
    condrem.setInputCloud(cloud.makeShared());
    condrem.filter(cloud);
    
    auto filtered_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *filtered_pointcloud);
    filtered_pointcloud->header = msg->header;

    pointcloud_pub_->publish(*filtered_pointcloud);
  }

}