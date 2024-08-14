#ifndef TRACKING_STRATEGY_H_
#define TRACKING_STRATEGY_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <unordered_map>        
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/clock.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>


#include "geometry_msgs/msg/twist.hpp"
#include "include/common.h"
#include "include/param_node.h"
#include "include/context.hpp"
#include "person_tracking_msgs/msg/persons_info.hpp"
#include "include/message_sync_node.h"            // 新的订阅者头文件
// #include "geometry_msgs/msg/pose_stamped.hpp"

typedef struct {
  person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr first;
  sensor_msgs::msg::Image::ConstSharedPtr second;
  sensor_msgs::msg::LaserScan::ConstSharedPtr third;
} Triplet;



// #include "include/robot_ctrl_node.h" // 准备删除

class TrackingManager : public rclcpp::Node {
 public:
  static std::shared_ptr<TrackingManager> Instance();
  ~TrackingManager();
  void byl_log(std::string const & text);

  
  void Release();


//   void FeedSmart1(const person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr &msg, const sensor_msgs::msg::Image::ConstSharedPtr &image);
  void FeedSmart2(const person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr &msg, const sensor_msgs::msg::Image::ConstSharedPtr &image, const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg);


  std::vector<std::shared_ptr<rclcpp::Node>> GetNodes();
  const TrackCfg &GetTrackCfg() const;

 private:
  TrackingManager();

  
//   void ProcessSmart(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image);
  void ProcessSmart(const Triplet &msg_image);


  // 做可视化，把要跟踪的行人在图像中标记出来，发布出去
//   void Visualization(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image);
  void Visualization(const Triplet &msg_image);

//   void Publish_goal(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image);
  void Caculate_goal(const Triplet &msg_image);

  void CancelMove();

  std::mutex robot_strategy_mtx_;
  // 如果上一帧智能数据的策略未处理完，当前智能数据不处理 
  std::atomic_bool last_frame_done_;

  std::shared_ptr<ParametersClass> param_node_ = nullptr;

  TrackInfo track_info_;
  TrackCfg track_cfg_;

  bool start_ = false;

  size_t queue_len_limit_ = 5;

  //需要增加一个queue，包含ai_msgs和image，pair类型的queue
  // std::queue<std::unordered_map<ai_msgs::msg::PerceptionTargets::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> > smart_image_queue_;
  // std::queue<std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> > smart_image_queue_;
  std::queue<Triplet> smart_image_queue_;

  std::mutex smart_queue_mtx_;
  std::condition_variable smart_queue_condition_;

  std::shared_ptr<std::thread> smart_process_task_ = nullptr;

  bool last_cmdvel_is_cancel_ = false;
  // 0: rotate, 1: move, 2: rotate&move
  int last_cmdvel_type_ = -1;
  float last_move_step_ratio_ = 1.0;


  std::unique_ptr<Context> context;  // 加入一个Context类型的指针
  int num_pos_samples = 0;           // 用于在TRAINING状态下计数（喂入正样本的数量）
  std::mutex context_mutex;
  std::unordered_map<long unsigned int, int> positive_count;   // 用于在LOST状态时，记录相应id作为正样本出现的次数


  // test发布者，运行过程中发布相关消息，调试用
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_publisher_; 

  // 可视化图片发布者
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visual_publisher_; 

  // 目标指令发布者
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;

  // /pose消息的订阅者
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  

  // tf buffer和listener
  // tf2_ros::Buffer::SharedPtr tf_buffer_;
  // tf2_ros::TransformListener::SharedPtr tf_listener_;



  // 是否发布可视化消息
  bool if_visualization = true;

  std::unordered_map<TrackingStatus, std::string> state_;

  // 当前的/goal_pose消息
  geometry_msgs::msg::PoseStamped my_goal;
  // 上次发布/goal_pose消息的时间
  // rclcpp::Time last_publish_time_{rclcpp::Clock().now()};
  rclcpp::Time last_publish_time_{rclcpp::Clock().now()};

};


#endif
