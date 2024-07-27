#include "hnurm_pointcloud/pointcloud_node.hpp"
#include<pcl_conversions/pcl_conversions.h>

#include<pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include<pcl/filters/passthrough.h>


namespace hnurm {

PointCloudNode::PointCloudNode():Node("pointcloud_node"){
  RCLCPP_INFO(get_logger(),"Pointcloud node created");
  declare_parameter("lidar_topic","/livox/lidar");     //segmentation/obstacle    /livox/lidar/pointcloud
  declare_parameter("pointcloud_output_topic","/pointcloud");
  declare_parameter("base_frame","base_footprint");
  declare_parameter("lidar_frame","livox_frame");
  // declare_parameter("scan_topic","/base_scan");
  // declare_parameter("scan_frame","base_scan");
}

void PointCloudNode::run(){
  RCLCPP_INFO(get_logger(),"Running pointcloud node");
  get_parameter("base_frame",base_frame_);
  get_parameter("lidar_frame",lidar_frame_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    get_parameter("lidar_topic").as_string(),
    rclcpp::SensorDataQoS(),
    std::bind(&PointCloudNode::pointcloud_callback,shared_from_this(),std::placeholders::_1)
  );
  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    get_parameter("pointcloud_output_topic").as_string(),
    rclcpp::SensorDataQoS()
  );
  // laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
  //   get_parameter("scan_topic").as_string(),
  //   rclcpp::SensorDataQoS()
  // );

}

void PointCloudNode::pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg){
  // RCLCPP_INFO(get_logger(),"Received pointcloud message");
  geometry_msgs::msg::TransformStamped transform;
  try{
    transform = tf_buffer_->lookupTransform(
      base_frame_,
      lidar_frame_,
      tf2::TimePointZero
    );
  }catch(tf2::TransformException &ex){
    RCLCPP_ERROR(get_logger(),"%s", ex.what());
    return;
  }
  sensor_msgs::msg::PointCloud2 transformed_pointcloud;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointCloud<pcl::PointXYZI> transformed_cloud;

  Eigen::Quaternionf q(
    transform.transform.rotation.w,
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z
  );
  Eigen::Affine3f t(Eigen::Translation3f(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z
  ) * q);

  pcl::transformPointCloud(cloud, transformed_cloud, t.matrix());

  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(transformed_cloud.makeShared());//makeshared可以优化,将transformed_cloud本身设为智能指针
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.3, 1.0);
  
  pass.filter(transformed_cloud);
  

  pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
  pcl::ConditionOr<pcl::PointXYZI>::Ptr condition(new pcl::ConditionOr<pcl::PointXYZI>());
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 0.4)));
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, -0.4)));
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, 0.4)));
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, -0.4)));
  
  
  condrem.setCondition(condition);
  condrem.setInputCloud(transformed_cloud.makeShared());
  condrem.filter(transformed_cloud);

  pcl::toROSMsg(transformed_cloud, transformed_pointcloud);
  transformed_pointcloud.header.frame_id = get_parameter("lidar_frame").as_string();
  pointcloud_pub_->publish(transformed_pointcloud);
  RCLCPP_INFO(get_logger(),"Published pointcloud message");
}




}