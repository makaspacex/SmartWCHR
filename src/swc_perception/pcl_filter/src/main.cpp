#include "pcl_filter/pointcloud_node.hpp"


int main(int argc,char* argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hnurm::PointCloudNode>();
  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
