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

#include "include/body_tracking.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include "include/time_helper.h"
#include "include/util.h"

std::shared_ptr<TrackingManager> TrackingManager::Instance() {
  static std::shared_ptr<TrackingManager> inst = nullptr;
  if (!inst) {
    inst = std::shared_ptr<TrackingManager>(new TrackingManager());
  }
  return inst;
}

TrackingManager::TrackingManager() {
  start_ = true;

  track_info_.is_movectrl_running = false;

  param_node_ = std::make_shared<ParametersClass>(&track_cfg_);

  robot_cmdvel_node_ =
      std::make_shared<RobotCmdVelNode>("horizon_tracking_RobotCmdVel");

  last_frame_done_ = true;


  context.reset(new Context());  // 初始化context
  num_pos_samples = 0;          // 初始化num_pos_samples



  state_[TrackingStatus::INITING] = "initing";
  state_[TrackingStatus::TRAINING] = "traininggggggggggggggggggggggg";
  state_[TrackingStatus::TRACKING] = "tracking";
  state_[TrackingStatus::LOST] = "lost";

  // 创建ROS节点
  auto node = rclcpp::Node::make_shared("publisher_node");
  // 初始化发布者
  // hello_publisher_ = node->create_publisher<std_msgs::msg::String>("hello_world", 10);

//   queue_publisher_ = node->create_publisher<std_msgs::msg::String>("queue_topic", 10);

//   strategy_publisher_ = node->create_publisher<std_msgs::msg::String>("strategy_topic", 10);

//   process_publisher_ = node->create_publisher<std_msgs::msg::String>("process_topic", 10);

  // sta_publisher_ = node->create_publisher<std_msgs::msg::String>("sta_topic", 10);

  test_publisher_ = node->create_publisher<std_msgs::msg::String>("test_topic", 10);

  visual_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("person_following_visual", 10);



//smart_image_queue_取代smart_queue_
  if (!smart_process_task_) {
    smart_process_task_ = std::make_shared<std::thread>([this]() {
      while (start_ && rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(smart_queue_mtx_);
        smart_queue_condition_.wait_for(lg, std::chrono::seconds(1), [&]() {
          // return !smart_queue_.empty() || !rclcpp::ok() || !start_;
          return !smart_image_queue_.empty() || !rclcpp::ok() || !start_;
        });
        if (smart_image_queue_.empty() || !rclcpp::ok() || !start_) {
          continue;
        }
        auto smart_frame = std::move(smart_image_queue_.front());
        smart_image_queue_.pop();
	
  /*
	// 调试用，发布话题，指示可以从队列中出消息
	auto queue_msg = std_msgs::msg::String();
	queue_msg.data = "queue pop " + std::to_string(queue_count_++);
	queue_publisher_->publish(queue_msg);
  */


        lg.unlock();
        RunTrackingStrategy(smart_frame);
      }
    });
  }
}

TrackingManager::~TrackingManager() {}

// 取消跟随
void TrackingManager::Release() {
  RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "TrackingManager release");
  start_ = false;

  if (smart_process_task_ && smart_process_task_->joinable()) {
    smart_process_task_->join();
    smart_process_task_ = nullptr;
  }

  param_node_ = nullptr;
  robot_cmdvel_node_ = nullptr;
}


void TrackingManager::UpdateTrackAngle() {
  if (track_info_.present_rect.empty()) return;

  cv::Point2f fit_center(track_cfg_.img_width / 2, track_cfg_.img_height);
  cv::Point2f start_pt(track_cfg_.img_width, track_cfg_.img_height);
  cv::Point2f end_pt(
      (track_info_.present_rect[0] + track_info_.present_rect[2]) / 2,
      (track_info_.present_rect[1] + track_info_.present_rect[3]) / 2);

  track_info_.angel_with_robot_ =
      CalAngelOfTwoVector(fit_center, start_pt, end_pt);

  {
    std::stringstream ss;
    ss << "frame_ts: " << track_info_.frame_ts
       << ", track_id: " << track_info_.track_id << ", angel_with_robot: "
       << std::abs(track_info_.angel_with_robot_ - 90)
       << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "UpdateTrackAngle: %s",
                ss.str().data());
  }

  auto transform_y_angle_to_x_radian =
      [this](float robot_y_negtive_angel) -> float {
    float robot_x_positive_angel = robot_y_negtive_angel;
    bool kps_is_clock_wise = true;
    if (robot_y_negtive_angel < 90) {
      kps_is_clock_wise = true;
      robot_x_positive_angel = 90 - robot_x_positive_angel;
    } else if (robot_y_negtive_angel > 90) {
      kps_is_clock_wise = false;
      robot_x_positive_angel = robot_x_positive_angel - 90;
    }
    float robot_x_positive_radian = robot_x_positive_angel * PI / 180.0;
    if (kps_is_clock_wise) {
      robot_x_positive_radian = (-1.0) * robot_x_positive_radian;
    }

    std::stringstream ss;
    ss << "robot robot_y_negtive_angel: " << robot_y_negtive_angel
       << ", robot_x_positive_angel: " << robot_x_positive_angel
       << ", robot_x_positive_radian: " << robot_x_positive_radian
       << ", kps_is_clock_wise: " << kps_is_clock_wise;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "%s", ss.str().c_str());
    return robot_x_positive_radian;
  };

  track_info_.robot_x_positive_radian_with_track_ =
      transform_y_angle_to_x_radian(track_info_.angel_with_robot_);
  track_info_.robot_y_negtive_radian_with_track_ =
      track_info_.angel_with_robot_ * PI / 180.0;
}

bool TrackingManager::RotateSwitch() {
  if (!last_cmdvel_is_cancel_ && 0 == last_cmdvel_type_) return true;

  if (track_info_.serial_lost_num > track_cfg_.track_serial_lost_num_thr) {
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "track_id: %d, serial_lost_num: %d, thr: %d, stop rotate!",
                track_info_.track_id,
                track_info_.serial_lost_num,
                track_cfg_.track_serial_lost_num_thr);
    return false;
  }

  // todo add lowpass strategy
  int activate_robot_rotate_thr = track_cfg_.activate_robot_rotate_thr;
  if (!track_info_.has_face_head) {
    activate_robot_rotate_thr = activate_robot_rotate_thr * 0.5;
  } else if (!last_cmdvel_is_cancel_ && 1 == last_cmdvel_type_) {
    activate_robot_rotate_thr = activate_robot_rotate_thr * 2;
  }
  if (std::abs(track_info_.angel_with_robot_ - 90) >=
      activate_robot_rotate_thr) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "RotateSwitchenable, activate_robot_rotate_thr: %d",
                activate_robot_rotate_thr);
    return true;
  }

  return false;
}




void TrackingManager::byl_log(std::string const & text)
{
  auto test_msg = std_msgs::msg::String();
  test_msg.data = ("[BYLLOG]:"+text).c_str();
  test_publisher_->publish(test_msg);
}




void TrackingManager::DoRotateMove() {
  if (!robot_cmdvel_node_) return;

  int direction = 1;
  float step = 0;
  auto twist = std::make_shared<Twist>();
  twist->linear.x = 0;
  twist->linear.y = 0;
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = 0;

  bool do_move = false;
  bool do_rotate = false;
  if (RotateSwitch()) {

    // byl_log("RotateSwich() return true;");

    do_rotate = true;
    // transform angel to yaw shift and wise
    int yaw_shift = 0;
    bool rotate_clock_wise = true;
    if (track_info_.angel_with_robot_ <= 90) {
      yaw_shift = 90 - track_info_.angel_with_robot_;
      rotate_clock_wise = true;
    } else if (track_info_.angel_with_robot_ <= 180) {
      yaw_shift = track_info_.angel_with_robot_ - 90;
      rotate_clock_wise = false;
    } else if (track_info_.angel_with_robot_ <= 270) {
      yaw_shift = track_info_.angel_with_robot_ - 90;
      rotate_clock_wise = false;
    } else if (track_info_.angel_with_robot_ <= 360) {
      yaw_shift = 360 - track_info_.angel_with_robot_ + 90;
      rotate_clock_wise = true;
    }

    float rotate_step_ratio = 1.0f;
    if (yaw_shift > track_cfg_.activate_robot_rotate_thr * 2 ||
        !track_info_.has_face_head) {
      rotate_step_ratio = 2.0;
    }
    float rotate_step = track_cfg_.rotate_step * rotate_step_ratio;
    std::stringstream ss;
    ss << "frame_ts: " << track_info_.frame_ts << ", yaw_shift: " << yaw_shift
       << ", rotate_clock_wise: " << rotate_clock_wise
       << ", rotate_step_ratio: " << rotate_step_ratio
       << ", rotate_step: " << rotate_step;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "rotate switch on: %s",
                ss.str().data());

    if (rotate_clock_wise) direction = 0;
    step = rotate_step;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "rotate direction: %d, step: %f",
                direction,
                step);
    int direct = 1;
    if (0 == direction) {
      direct = -1;
    }
    // 弧度＝度×π/180
    // twist->angular.z = direct * step * 3.14 / 180;
    twist->angular.z = direct * step;
  }

  if (TrackingSwitchWithVision()) {

    // byl_log("TrackingSwitchWithVision() return true;");

    do_move = true;
    direction = track_info_.move_direction;
    step = track_info_.move_step;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "move switch on, direction: %d, step: %f",
                direction,
                step);
    if (0 == direction) {
      twist->linear.x += step;
    } else if (1 == direction) {
      twist->linear.x -= step;
    } else if (2 == direction) {
      twist->linear.y += step;
    } else if (3 == direction) {
      twist->linear.y -= step;
    }
  }

  if (do_move || do_rotate) {
    last_cmdvel_is_cancel_ = false;
    last_cmdvel_type_ = 2;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "Do rotate move, ts sec: %llu, nanosec: %llu",
                track_info_.frame_ts_sec,
                track_info_.frame_ts_nanosec);
    robot_cmdvel_node_->RobotCtl(*twist);
    // 这个函数发布速度指令，加下来要看看*twist是怎么生成的

    // if (do_rotate) {
    //   // 避免由于智能结果输出时间间隔不均匀导致的小车过度运动
    //   static int cmd_ms = 30;
    //   std::this_thread::sleep_for(std::chrono::milliseconds(cmd_ms));
    //   CancelMove();
    // }
  } else {
    CancelMove();
  }
}

// 发布0速度指令，停止运动
void TrackingManager::CancelMove() {
  if (last_cmdvel_is_cancel_) return;
  if (robot_cmdvel_node_) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "cancel move");
    auto twist = std::make_shared<Twist>();
    twist->linear.x = 0;
    twist->linear.y = 0;
    twist->linear.z = 0;
    twist->angular.x = 0;
    twist->angular.y = 0;
    twist->angular.z = 0;
    robot_cmdvel_node_->RobotCtl(*twist);
  }

  last_cmdvel_is_cancel_ = true;
  last_cmdvel_type_ = -1;
  last_move_step_ratio_ = 1.0;
}

// 判断是否要动
bool TrackingManager::TrackingSwitchWithVision() {
  if (track_info_.present_rect.empty()) {
    // byl_log("[TrackingSwitchWithVision]: track_info_.present_rect.empty()  return false");
    return false;
  }

  // todo 20211230 如果没有face/head，认为距离很近，不需要move
  if (!track_info_.has_face_head) {
    // byl_log("[TrackingSwitchWithVision]: !track_info_.has_face_head  return false");

    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "track has no face and head, too close to robot");
    return false;
  }

  if (track_info_.present_rect[1] < track_cfg_.stop_robot_move_to_top_thr) {
    // byl_log("[TrackingSwitchWithVision]: track_info_.present_rect[1] < track_cfg_.stop_robot_move_to_top_thr");
    // 距离上边界很近，不需要move。如果移动可能拍不到face/head，导致检测不到body
    return false;
  }

  // 如果body rect宽度超过画面宽度一定比例，认为距离很近，不需要move
  int body_rect_width =
      track_info_.present_rect[2] - track_info_.present_rect[0];
  if (body_rect_width >=
      track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr) {
        // byl_log("[TrackingSwitchWithVision]: body_rect_width >= track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr");

    RCLCPP_INFO(
        rclcpp::get_logger("TrackingManager"),
        "track width: %d, exceeds %f of img_width_: %d, too close to robot",
        body_rect_width,
        track_cfg_.stop_move_rect_width_ratio_thr,
        track_cfg_.img_width);
    return false;
  }

  // 根据body rect宽度，计算move step，宽度越小，step越大
  track_info_.move_step = track_cfg_.move_step;
  float move_step_ratio = 1.0;
  int body_rect_to_top = track_info_.present_rect[1];
  // 只有当robot和target之间的角度较小时才可以加速move，否则可能导致robot撞倒障碍物或者跟丢target
  if (abs(track_info_.robot_x_positive_radian_with_track_ * 180 / PI) <
      track_cfg_.activate_robot_rotate_thr) {
    if (body_rect_to_top > track_cfg_.img_height * 0.5) {
      move_step_ratio = 4;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.45) {
      move_step_ratio = 3.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.4) {
      move_step_ratio = 3.0;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.35) {
      move_step_ratio = 2.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.3) {
      move_step_ratio = 2.0;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.25) {
      move_step_ratio = 1.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.2) {
      move_step_ratio = 1.2;
    }
  }

  if (move_step_ratio > (last_move_step_ratio_ + 0.5)) {
    // 限制速度，避免突然速度
    move_step_ratio = (last_move_step_ratio_ + 0.5);
    last_move_step_ratio_ = move_step_ratio;
  }

  track_info_.move_step = move_step_ratio * track_cfg_.move_step;

  // 只根据body rect宽度判断是否需要move 20220119
  int to_top_thr =
      track_cfg_.activate_robot_move_thr;  // track_cfg_.img_height * 0.1;
  if (body_rect_width <
          track_cfg_.img_width * track_cfg_.start_move_rect_width_ratio_thr &&
      body_rect_to_top > to_top_thr) {
    std::stringstream ss;
    ss << "Do move! body_rect_width: " << body_rect_width << ", thr: "
       << track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr
       << ", move_step_ratio: " << move_step_ratio
       << ", body_rect_to_top: " << body_rect_to_top
       << ", img_height: " << track_cfg_.img_height
       << ", move_step: " << track_info_.move_step;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "%s", ss.str().c_str());
    return true;
  } else {
    std::stringstream ss;
    ss << "Do not move! body_rect_width: " << body_rect_width << ", thr: "
       << track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr
       << ", move_step_ratio: " << move_step_ratio
       << ", body_rect_to_top: " << body_rect_to_top
       << ", img_height: " << track_cfg_.img_height
       << ", move_step: " << track_info_.move_step;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "%s", ss.str().c_str());
  }
  // byl_log("[TrackingSwitchWithVision]: Do not move!");
  return false;
}


// 在这个函数当中，完成四个状态转换的逻辑
// initial、training、Lost(reid)、tracking
void TrackingManager::ProcessSmart(
    std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
  

  auto msg = msg_image.first;  // msg是订阅到的PeronsInfo

  if (!msg || !rclcpp::ok()) {
    return;
  }

  // cv::Mat cv_image = cv::imdecode(cv::Mat(msg_image.second->data), cv::IMREAD_COLOR);
  cv::Mat cv_image = cv_bridge::toCvCopy(msg_image.second, "bgr8")->image;

  
  auto image_size = cv_image.size();
  /*
  auto image_size_msg = "width：" + std::to_string(image_size.width) + ", height: " + std::to_string(image_size.height);
  byl_log("image_size：" + image_size_msg);
  */
  

  // byl_log("At the beginning of ProcessSmart, the state is " + state_[track_info_.tracking_sta]);
    

  uint64_t frame_ts = (msg->header.stamp.sec % 1000) * 1000 +
                      msg->header.stamp.nanosec / 1000 / 1000;

  track_info_.frame_ts_sec = msg->header.stamp.sec;
  track_info_.frame_ts_nanosec = msg->header.stamp.nanosec;

  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "process smart frame_ts %llu",
              frame_ts);

  // 在开始状态判断之前，首先把所有检测到的人体框以及对应的track_id构成一个unordered_map，用于后续进行特征提取和重识别
  std::unordered_map<long unsigned int, Tracklet::Ptr> tracks;

  for (const auto &person : msg->persons) {
    tracks[person.id].reset(new Tracklet());

    float center_x = person.center.x;
    float center_y = person.center.y;
    float width = person.width;
    float height = person.height;

    // tl和br分别代表检测框左上角和右下角的坐标
    cv::Point tl(center_x - width / 2, center_y - height / 2);
    cv::Point br(center_x + width / 2, center_y + height / 2);
    
    tl.x = std::min(image_size.width, std::max(0, tl.x));
    tl.y = std::min(image_size.height, std::max(0, tl.y));
    br.x = std::min(image_size.width, std::max(0, br.x));
    br.y = std::min(image_size.height, std::max(0, br.y));

    cv::Rect body_region(tl, br);

    tracks[person.id]->person_region = body_region;
    tracks[person.id]->id = person.id;
  }

  // context的互斥锁
  std::lock_guard<std::mutex> lock(context_mutex);
  // 提取所有body的特征
  context->extract_features(cv_image, tracks);      
  

  // 改成四种状态，INITING、TRAINING、TRACKING和LOST(reid)
  if (TrackingStatus::TRACKING == track_info_.tracking_sta) { 
    // TRACKING的状态下，在所有检测到的对象中寻找id是target的对象       
    // update status
    // find track
    /*
    在所有检测框中寻找target
    ①没找到，进入LOST状态
    ②找到了，根据分类器对于target的置信度进行情况分类
      1、对于target的置信度高于一定值，用当前的正负样本更新分类器，状态不变
      2、置信度大于0但是低于一定阈值，不更新分类器，状态不变
      3、置信度小于0，说明target_id的检测框中不再是目标跟踪行人，进入LOST状态
    同时检测所有的手势，检查目前的target是否做出了取消跟随的手势，如果是，进入INITING状态
    要更新context分类器以及track_info
    */
    
    // byl_log("come in the tracking state..........");


    bool stay_TRACKING = true;               // 指示是否还停留在TRACKING状态                   
    auto found = tracks.find(track_info_.track_id);

    if(found == tracks.end()) {                          // TRACKING状态中跟丢了目标，进入LOST状态 
      RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "lost target in TRAINING STATE");
      track_info_.tracking_sta = TrackingStatus::LOST;  
      stay_TRACKING = false; 

    } else {
      boost::optional<double> pred = context->predict(found->second);   // 用分类器得到target框的置信度
      if(pred && *pred < -0.1) {                                      // 置信度小于-0.1，说明目标跟丢了，还是进入LOST状态
        RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "ID switch detected!!");
        track_info_.tracking_sta = TrackingStatus::LOST;   
        stay_TRACKING = false;

      } else if(!pred || *pred < 0.1) {                              // 置信度小于0.1，没跟丢但是质量不好，不更新分类器
        RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "do not update");

      } else {                                                       // 更新分类器
        for(const auto& track: tracks) {
          double label = track.first == track_info_.track_id ? 1.0 : -1.0;

          context->update_classifier(label, track.second);

        }
      }
    }
    
    // 如果没跟丢，即stay_TRACKING == true，则要进行下面的代码，判断跟踪对象是否做出了取消跟随的指令
    if (stay_TRACKING) {
      bool cancel = false;
      
      // 如果跟随目标左右手都举起来了，停止跟随
      for (const auto &person : msg->persons) {
        if(person.id != track_info_.track_id)  continue;

        // 左右手同时举起，停止跟随
        // (person.keypoints[11].x > 0 && person.keypoints[15].x > 0) && (person.keypoints[11].y > person.keypoints[15].y) &&
        if ((person.keypoints[12].x > 0 && person.keypoints[16].x > 0) && (person.keypoints[12].y > person.keypoints[16].y)) {
              cancel = true;
              byl_log("cancel is true");
              break;
        }
      } 

      if (cancel) { 
      // byl_log("set the tracking_sta to INITINGGGGGGGGGGGGGGGGGGGGG");
      track_info_.tracking_sta = TrackingStatus::INITING;     // 设置状态为initing，初始状态，重新寻找跟随的人
      track_info_.track_id = -1;
      }

    }

    
  }


  else if (TrackingStatus::INITING == track_info_.tracking_sta) {  // INITING状态，主要确定跟随的对象

    if (msg->persons.empty()) {                          // 如果没有找到任何目标框，直接return
      return;
    }

    // 中途进入INITING的两种情况，要么是TRAINING过程中目标没了，要么是目标发出取消跟随的指令，都需要重新初始化分类器
    context.reset(new Context());  // 重新初始化context

    // 在INITING或者LOST状态下检测到了检测框 
    // 各种变量初始化


    bool find_track = false;
    uint64_t track_id;
    std::vector<int> body_rect;

    for (const auto &person : msg->persons) {
      // TODO
      /*增加判断条件，确定跟随对象*/

      // 
      if ((person.keypoints[11].x > 0 && person.keypoints[15].x > 0) && (person.keypoints[11].y > person.keypoints[15].y)) {
        find_track = true;
      }
      
      if (find_track) {
        body_rect.push_back(person.center.x - person.width / 2);
        body_rect.push_back(person.center.y - person.height / 2);
        body_rect.push_back(person.center.x + person.width / 2);
        body_rect.push_back(person.center.y + person.height / 2);
        track_id = person.id;

        // find_track = true;
        break;
      }
    }

    // 到此，如果找到了目标人体，find_track==true，track_id是目标人体检测框的id，present_track_rect是目标人体检测框

    if (find_track) {           // 在INITIAL状态下找到了正确做出激活手势的人
      memset(static_cast<void *>(&track_info_), 0, sizeof(TrackInfo));  // 重置track_info_
      
      track_info_.tracking_sta = TrackingStatus::TRAINING;              // 将状态设为正在跟踪，设置正在跟踪的检测框的id、框等
      // byl_log("find_track is true, find the people with right gesture, set the tracking_sta to TRAINING");     
      
      track_info_.track_id = track_id;
      track_info_.present_rect = body_rect;
      track_info_.frame_ts = frame_ts;
    }

  } 
  
  
  else if (TrackingStatus::TRAINING == track_info_.tracking_sta) {
    // byl_log("come in the training sta.......");

    // 在当前帧中寻找与目标检测框id一致的检测框
    auto found = tracks.find(track_info_.track_id);
    if(found == tracks.end()) {
      
      track_info_.tracking_sta = TrackingStatus::INITING;   // 没找到target_id回到INITING状态
      num_pos_samples = 0;                                 // 离开TRAINING状态时将num_pos_samples重新初始化成0
      RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "lost target during the initial training!!");

    } else if(num_pos_samples < 10){   // 这个阈值设置成10，当不足10张正样本照片喂入的时候，先不跟随，只训练分类器
      for(const auto& track: tracks) {
        double label = track.first == track_info_.track_id ? 1.0 : -1.0;
        boost::optional<double> pred = context->predict(track.second);

        context->update_classifier(label, track.second);

        if(label > 0.0) {
            num_pos_samples ++;
        }
      }   
    } else { // 超过10张正样本照片喂入，开始跟随，进入TRACKING状态，离开TRAINING状态把num_pos_samples重新初始化
        track_info_.tracking_sta = TrackingStatus::TRACKING;
        num_pos_samples = 0;  
      }

  } 
  
  
  
  else if (TrackingStatus::LOST == track_info_.tracking_sta) {
    // byl_log("come in the LOST state..............." );


    long target_id = -1;
    for(const auto& track: tracks) {
        // byl_log("In the LOST sta, the tracks is not NULL" );
        boost::optional<double> pred = context->predict(track.second);       // 对每个行人框都用分类器进行预测
        
        if(!pred || pred < 0.2) {                                           // 置信度小于0.2，continue
            continue;
        }

        auto found = positive_count.find(track.first);
        if(found == positive_count.end()) {
            positive_count[track.first] = 0;
        }
        positive_count[track.first] ++;

        if(positive_count[track.first] >= 5) {
          target_id = track.first;
          break;
        }
    }
    // 如果target_id > 0，说明id为target_id的target作为正样本出现了5次，可以作为继续跟随的目标
    // 接下来进入TRACKING状态

    if(target_id > 0) {
      
      // byl_log("In LOST sta, set the sta to TRACKING");

      memset(static_cast<void *>(&track_info_), 0, sizeof(TrackInfo));  // 重置track_info_
      track_info_.tracking_sta = TrackingStatus::TRACKING;              // 重新回到TRACKING状态，设置正在跟踪的检测框的id、框等
      track_info_.track_id = (uint64_t)target_id;
      track_info_.frame_ts = frame_ts;
      


      for (const auto &person : msg->persons) {  // 遍历检测到的所有targets
        if(person.id == target_id) {       // 找到对应的目标target
          std::vector<int> body_rect;

          body_rect.push_back(person.center.x - person.width / 2);
          body_rect.push_back(person.center.y - person.height / 2);
          body_rect.push_back(person.center.x + person.width / 2);
          body_rect.push_back(person.center.y + person.height / 2);
          track_info_.present_rect = body_rect;
          break;
        }
      } 

      //离开LOST状态，要把positive_count重置
      positive_count = std::unordered_map<long unsigned int, int>();
    }
  }

}

void TrackingManager::RunTrackLostProtectionStrategy() {
  // 人脸或者头部消失时间大于20，说明目标距离机器人很近，停止移动
  if (track_info_.serial_lost_face_head_num >= 20) {
    // start protection strategy
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "track lost warnning! serial_lost_face_head_num: %d",
                track_info_.serial_lost_face_head_num);
    // cancel move goal
    CancelMove();
  }
  return;

  int angle_with_track = std::abs(track_info_.angel_with_robot_ - 90);
  // start protection strategy
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "The angle between robot and track is %d, limit is %d",
              angle_with_track,
              track_cfg_.track_lost_protection_angel_thr);

  if (angle_with_track < track_cfg_.track_lost_protection_angel_thr) {
    return;
  }

  // start protection strategy
  RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
              "track lost warnning! The angle between robot and track is %d "
              "exceeds limit %d, track lost protection strategy is activated!",
              std::abs(track_info_.angel_with_robot_ - 90),
              track_cfg_.track_lost_protection_angel_thr);
  // cancel move goal
  CancelMove();
}

void TrackingManager::RunOverMovingProtectionStrategy() {
  if (!last_cmdvel_is_cancel_ && 0 == last_cmdvel_type_ &&
      (std::abs(track_info_.angel_with_robot_ - 90) <
       track_cfg_.track_overmoving_protection_angel_thr)) {
    // start protection strategy
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "frame_ts %llu, robot over moving warnning! The angle between "
                "robot and track is %d, less than limit %d, robot overmoving "
                "strategy is activated!",
                track_info_.frame_ts,
                std::abs(track_info_.angel_with_robot_ - 90),
                track_cfg_.track_overmoving_protection_angel_thr);
    CancelMove();
  }
}

// 不支持避障的跟随策略，适用于只有camera，没有imu lidar等传感器的情况
void TrackingManager::TrackingWithoutNavStrategy(
    std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {



  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),  "Run TrackingWithoutNav strategy");

  if (!last_frame_done_) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "last frame is not done");
    return;
  }

  // 1. update track info
  ProcessSmart(msg_image);

  if (if_visualization) {
    Visualization(msg_image);
  }
  

  last_frame_done_ = false;

  
  if (TrackingStatus::INITING == track_info_.tracking_sta) {
    CancelMove();
    last_frame_done_ = true;
    return;
  }




  // 这里，如果不是TRACKING状态，就置回INITING，这样TRAINING状态也会被置成INITING
  // 这里需要修改
  // 原本是只有INITING、TRACKING和LOST三种状态，在ProcessSmart函数中，INITING和LOST进行同样的操作
  // 其实在原本的代码中，应该只有LOST状态下会触发下面的if

  
  if (TrackingStatus::TRACKING != track_info_.tracking_sta) {
    // RCLCPP_DEBUG(rclcpp::get_logger("TrackingManager"), "track is lost");
    CancelMove();
    last_frame_done_ = true;
    return;
  }



  // 2. cal the angle of robot and track
  UpdateTrackAngle();

  if (!last_cmdvel_is_cancel_) {
    RunOverMovingProtectionStrategy();
  }

  RunTrackLostProtectionStrategy();
  DoRotateMove();
  last_frame_done_ = true;
  return;
}


void TrackingManager::RunTrackingStrategy(
    std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
  std::unique_lock<std::mutex> lg(robot_strategy_mtx_);

  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "Run TrackingStrategy");
  {
    static auto last_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();
    auto present_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "Run TrackingStrategy time ms diff: %llu",
                present_tracking_ts - last_tracking_ts);
    last_tracking_ts = present_tracking_ts;
  }

  auto start_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();

  // 只使用视觉感知结果实现跟随，不支持避障
  TrackingWithoutNavStrategy(msg_image);

  auto present_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "Run TrackingStrategy time ms cost: %llu",
              present_tracking_ts - start_tracking_ts);

  return;
}



// 增加的FeedSmart1代替FeedSmart
void TrackingManager::FeedSmart1(const person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr &msg, const sensor_msgs::msg::Image::ConstSharedPtr &image) {
  if (!rclcpp::ok()) return;
  std::unique_lock<std::mutex> lg(smart_queue_mtx_);

  // std::unordered_map<ai_msgs::msg::PerceptionTargets::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> tmp;
  // tmp[msg] = image;
  // std::pair<int, double> pair1(42, 3.14);
  std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> tmp(msg, image);
  smart_image_queue_.push(tmp);

  if (smart_image_queue_.size() > queue_len_limit_) {
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "smart queue len exceed limit: %d",
                queue_len_limit_);
    smart_image_queue_.pop();
  }
  smart_queue_condition_.notify_one();
  lg.unlock();
}

std::vector<std::shared_ptr<rclcpp::Node>> TrackingManager::GetNodes() {
  std::vector<std::shared_ptr<rclcpp::Node>> node_ptrs;
  node_ptrs.push_back(param_node_);
  node_ptrs.push_back(robot_cmdvel_node_);
  return node_ptrs;
}

const TrackCfg &TrackingManager::GetTrackCfg() const { return track_cfg_; }


void TrackingManager::Visualization(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
  // cv::Mat input_image = cv::imdecode(cv::Mat(msg_image.second->data), cv::IMREAD_COLOR);
  cv::Mat input_image = cv_bridge::toCvCopy(msg_image.second, "bgr8")->image;

  auto msg = msg_image.first;  // msg是订阅到的PeronsInfo

  if (!msg || !rclcpp::ok()) {
    return;
  }


  for (const auto &person : msg->persons) {

    float center_x = person.center.x;
    float center_y = person.center.y;
    float width = person.width;
    float height = person.height;

    // 计算检测框的左上角和右下角坐标
    int x1 = static_cast<int>(center_x - width / 2);
    int y1 = static_cast<int>(center_y - height / 2);
    int x2 = static_cast<int>(center_x + width / 2);
    int y2 = static_cast<int>(center_y + height / 2);

    // 绘制矩形框
    if (person.id == track_info_.track_id) {
      cv::rectangle(input_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);  // 目标画红框
    } else {
      cv::rectangle(input_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);  // 非目标画绿框
    }
  }


  // Convert cv::Mat to ROS image message
  auto cv_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", input_image).toImageMsg();
  // Publish image
  visual_publisher_->publish(*cv_msg);


  /*
  auto image_size = input_image.size();
  auto image_size_msg = "input_size:  width：" + std::to_string(image_size.width) + ", height: " + std::to_string(image_size.height);
  byl_log(image_size_msg);
  */


}
