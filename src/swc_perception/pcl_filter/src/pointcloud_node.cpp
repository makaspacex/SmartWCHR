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
    
    // 检查点云消息是否包含强度字段
    bool has_intensity = false;

    for (const auto& field : msg->fields)
    {
        if (field.name == "intensity")
        {
            has_intensity = true;
            break;
        }
    }

    pcl::PointCloud<pcl::PointXYZI> cloud_with_intensity;

    if (has_intensity)
    {
        // 如果消息中已经有强度信息，则直接转换
        pcl::fromROSMsg(*msg, cloud_with_intensity);
    }
    else
    {
        // 创建一个新的点云，包含强度信息
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        cloud_with_intensity.resize(cloud.size());

        // 复制点云数据并设置强度值（这里设置为默认值10）
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            cloud_with_intensity.points[i].x = cloud.points[i].x;
            cloud_with_intensity.points[i].y = cloud.points[i].y;
            cloud_with_intensity.points[i].z = cloud.points[i].z;
            cloud_with_intensity.points[i].intensity = 10.0f; // 或者设置为其他值
        }
    }


    // 使用Passthrough滤波器对点云进行裁剪
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud_with_intensity.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.6);
    pass.filter(cloud_with_intensity);

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
    condrem.setInputCloud(cloud_with_intensity.makeShared());
    condrem.filter(cloud_with_intensity);

    // 将裁剪后的点云转换回ROS消息格式
    sensor_msgs::msg::PointCloud2 filtered_pointcloud;
    pcl::toROSMsg(cloud_with_intensity, filtered_pointcloud);
    filtered_pointcloud.header.frame_id = "livox_frame";

    // 发布裁剪后的点云消息
    pointcloud_pub_->publish(filtered_pointcloud);
  }

}