// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRACKING_STRATEGY_H_
#define TRACKING_STRATEGY_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <unordered_map>         // 新加的头文件
#include <utility>               // 新加的头文件
#include "include/context.hpp"

#include "person_tracking_msgs/msg/persons_info.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "include/common.h"
#include "include/param_node.h"
#include "include/robot_ctrl_node.h"
// #include "include/smart_subscriber.h"  原始的订阅者头文件，不用包含了


// #include "include/sub.h"
// #include "include/pub.h"
#include "include/message_sync_node.h"            // 新的订阅者头文件
// #include "include/sub_topic.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"


class TrackingManager {
 public:
  static std::shared_ptr<TrackingManager> Instance();
  ~TrackingManager();
  void byl_log(std::string const & text);

  
  void Release();

  // 需要增加一个FeedSmart函数，包含两个参数而不是一个，将ai_msgs和image作为key-value对加入队列
  // 20230828 应该不用作为key-value对存到unordered map这种数据结构里，直接作为pair就行
  void FeedSmart1(const person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr &msg, const sensor_msgs::msg::Image::ConstSharedPtr &image);

  std::vector<std::shared_ptr<rclcpp::Node>> GetNodes();
  const TrackCfg &GetTrackCfg() const;

 private:
  TrackingManager();

  // 原本参数是ai_msg类型消息指针的函数，现在要改成是
  // std::unordered_map<ai_msgs::msg::PerceptionTargets::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr>类型的msg_image
  // 可以在函数里面令msg = msg_image.first
  

  // void RunTrackingStrategy(
  //     const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  void RunTrackingStrategy(
       std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image);


  // void TrackingWithoutNavStrategy(
  //     const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);

  void TrackingWithoutNavStrategy(
      std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image);

  // avoid track lost from camera on robot
  void RunTrackLostProtectionStrategy();
  // avoid robot move far away
  void RunOverMovingProtectionStrategy();

  
  // void ProcessSmart(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  void ProcessSmart(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image);

  // 做可视化，把要跟踪的行人在图像中标记出来，发布出去
  void Visualization(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image);

  // check if last command send to robot is complete
  bool RobotCommandIsRunning();
  // cal the angle of robot and track
  void UpdateTrackAngle();
  // check if robot needs rotate
  bool RotateSwitch();

  void CancelMove();

  void DoRotateMove();

  // check if robot needs move
  // 只使用视觉检测结果数据，判断是否需要激活跟随
  bool TrackingSwitchWithVision();

  std::mutex robot_strategy_mtx_;
  // 如果上一帧智能数据的策略未处理完，当前智能数据不处理 
  std::atomic_bool last_frame_done_;

  std::shared_ptr<ParametersClass> param_node_ = nullptr;
  std::shared_ptr<RobotCmdVelNode> robot_cmdvel_node_ = nullptr;

  TrackInfo track_info_;
  TrackCfg track_cfg_;

  bool start_ = false;

  // size_t queue_len_limit_ = 2;   把这个数值改大一点试试
  size_t queue_len_limit_ = 5;
  // std::queue<ai_msgs::msg::PerceptionTargets::ConstSharedPtr> smart_queue_;

  //需要增加一个queue，包含ai_msgs和image，pair类型的queue
  // std::queue<std::unordered_map<ai_msgs::msg::PerceptionTargets::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> > smart_image_queue_;
  std::queue<std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> > smart_image_queue_;

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



  // 在从quque_pop的地方发布
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr queue_publisher_;
//   long queue_count_ = 0;
  
  // 在调用TrackingWithoutNavStrategy函数的时候发布消息
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr strategy_publisher_;
//   long strategy_count_ = 0;


  // 在调用ProcessSmart的时候发布
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr process_publisher_; 
//   long process_count_ = 0;

  // 在ProcessSmart函数中发布，指示当前的状态
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sta_publisher_; 


  // test发布者，运行过程中发布相关消息，调试用
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_publisher_; 

  // 可视化图片发布者
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visual_publisher_; 

  // 是否发布可视化消息
  bool if_visualization = true;


  
  
  // std::string state_[4] = {"INITING", "TRAINING", "TRACKING", "LOST" };
  std::unordered_map<TrackingStatus, std::string> state_;
};




#endif
