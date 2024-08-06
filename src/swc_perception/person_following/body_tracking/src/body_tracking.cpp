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

TrackingManager::TrackingManager() : Node("body_tracking") {
  start_ = true;

  track_info_.is_movectrl_running = false;

  param_node_ = std::make_shared<ParametersClass>(&track_cfg_);

//   robot_cmdvel_node_ =
//       std::make_shared<RobotCmdVelNode>("horizon_tracking_RobotCmdVel");

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
  test_publisher_ = node->create_publisher<std_msgs::msg::String>("test_topic", 10);
  // 可视化跟踪结果
  visual_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("person_following_visual", 10);
  // /goal消息发布者
  goal_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);

  // 创建 tf2 变换监听器
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



// 行人检测框、原始图片消息出队
  if (!smart_process_task_) {
    smart_process_task_ = std::make_shared<std::thread>([this]() {
      while (start_ && rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(smart_queue_mtx_);
        smart_queue_condition_.wait_for(lg, std::chrono::seconds(1), [&]() {
          return !smart_image_queue_.empty() || !rclcpp::ok() || !start_;
        });
        if (smart_image_queue_.empty() || !rclcpp::ok() || !start_) {
          continue;
        }
        auto smart_frame = std::move(smart_image_queue_.front());
        smart_image_queue_.pop();
	
        lg.unlock();

        // RunTrackingStrategy(smart_frame);

        ProcessSmart(smart_frame);

        if (if_visualization) {
          Visualization(smart_frame);
        }

        if (track_info_.tracking_sta == TrackingStatus::TRACKING) {
          // 发布/goal消息，跟随目标行人
          Publish_goal(smart_frame);
        }

      }
    });
  }
}

TrackingManager::~TrackingManager() {}

// 取消跟随    是一个思路，通过一个变量或者指针来控制是否启动跟随
void TrackingManager::Release() {
  RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "TrackingManager release");
  start_ = false;

  if (smart_process_task_ && smart_process_task_->joinable()) {
    smart_process_task_->join();
    smart_process_task_ = nullptr;
  }

  param_node_ = nullptr;
//   robot_cmdvel_node_ = nullptr;
}


void TrackingManager::byl_log(std::string const & text)
{
  auto test_msg = std_msgs::msg::String();
  test_msg.data = ("[BYLLOG]:"+text).c_str();
  test_publisher_->publish(test_msg);
}



// 发布0速度指令，停止运动
void TrackingManager::CancelMove() {

}

// 在这个函数当中，完成四个状态转换的逻辑
// initial、training、Lost(reid)、tracking
void TrackingManager::ProcessSmart(
    std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
  

  auto msg = msg_image.first;  // msg是订阅到的PeronsInfo

  if (!msg || !rclcpp::ok()) {
    return;
  }

  cv::Mat cv_image = cv_bridge::toCvCopy(msg_image.second, "bgr8")->image;

  
  auto image_size = cv_image.size();
    


  uint64_t frame_ts = (msg->header.stamp.sec % 1000) * 1000 +
                      msg->header.stamp.nanosec / 1000 / 1000;

  track_info_.frame_ts_sec = msg->header.stamp.sec;
  track_info_.frame_ts_nanosec = msg->header.stamp.nanosec;
/*
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "process smart frame_ts %llu",
              frame_ts);
*/

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

    } 
    

    // 注释掉这段在跟踪过程中更新分类器的代码，一开始输入10张训练照片就不更新分类器了
    // 因为在跟随的过程中更新分类器速率太慢了
    /*
    else {
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
    */
    
    // 如果没跟丢，即stay_TRACKING == true，则要进行下面的代码，判断跟踪对象是否做出了取消跟随的指令
    if (stay_TRACKING) {
      bool cancel = false;
      
      for (const auto &person : msg->persons) {
        if(person.id != track_info_.track_id)  continue;

        // 举起右手，取消跟随
        if ((person.keypoints[12].x > 0 && person.keypoints[16].x > 0) && (person.keypoints[12].y > person.keypoints[16].y)) {
              cancel = true;
              byl_log("cancel is true");
              break;
        }
      } 

      if (cancel) { 
      track_info_.tracking_sta = TrackingStatus::INITING;     // 设置状态为initing，初始状态，重新寻找跟随的人
      track_info_.track_id = -1;
      }

    }
  }

  else if (TrackingStatus::INITING == track_info_.tracking_sta) {  // INITING状态，主要确定跟随的对象

    if (msg->persons.empty()) {                          // 如果没有找到任何目标框，直接return
      return;
    }

    // 在INITING或者LOST状态下检测到了检测框 
    // 各种变量初始化


    bool find_track = false;
    uint64_t track_id;
    std::vector<int> body_rect;

    for (const auto &person : msg->persons) {
      // TODO
      /*增加判断条件，确定跟随对象*/

      // 举起左手，确定为跟随对象
      if ((person.keypoints[11].x > 0 && person.keypoints[15].x > 0) && (person.keypoints[11].y > person.keypoints[15].y)) {
        find_track = true;
      }
      
      if (find_track) {
        body_rect.push_back(person.center.x - person.width / 2);
        body_rect.push_back(person.center.y - person.height / 2);
        body_rect.push_back(person.center.x + person.width / 2);
        body_rect.push_back(person.center.y + person.height / 2);
        track_id = person.id;

        break;
      }
    }

    // 到此，如果找到了目标人体，find_track==true，track_id是目标人体检测框的id，present_track_rect是目标人体检测框

    if (find_track) {           // 在INITIAL状态下找到了正确做出激活姿势的人
      memset(static_cast<void *>(&track_info_), 0, sizeof(TrackInfo));  // 重置track_info_
      
      // 重新初始化分类器
      context.reset(new Context());  // 重新初始化context
      
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


// 增加的FeedSmart1代替FeedSmart
void TrackingManager::FeedSmart1(const person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr &msg, const sensor_msgs::msg::Image::ConstSharedPtr &image) {
  if (!rclcpp::ok()) return;
  std::unique_lock<std::mutex> lg(smart_queue_mtx_);

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
//   node_ptrs.push_back(robot_cmdvel_node_);
  return node_ptrs;
}

const TrackCfg &TrackingManager::GetTrackCfg() const { return track_cfg_; }


void TrackingManager::Visualization(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
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
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "publish a visualization image");
  */
 
}


void TrackingManager::Publish_goal(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
  cv::Mat input_image = cv_bridge::toCvCopy(msg_image.second, "bgr8")->image;

  auto msg = msg_image.first;  // msg是订阅到的PeronsInfo
  if (!msg || !rclcpp::ok()) {
    return;
  }

  cv::Point2f center(-1, -1);   //  = cv::Point2f
  for (const auto &person : msg->persons) {
    if (person.id == track_info_.track_id) {
      center.x = person.center.x;
      center.y = person.center.y;
    }
  }

  if (center.x > 0) {
    // 获取相应坐标的深度值
    // float depth_value = extract_depth_from_pointcloud(msg, image_x, image_y);
    float depth_value = 1.0;

    
    /*
      data: [536.75638,   0.     , 286.90511,
           0.     , 537.00102, 284.33105,
           0.     ,   0.     ,   1.     ]
    */
    // 相机内参
    float fx = 536.75638; // Example focal length in pixels (x-direction)
    float fy = 537.00102; // Example focal length in pixels (y-direction)
    float cx = 286.90511; // Example principal point (x-coordinate)
    float cy = 284.33105; // Example principal point (y-coordinate)

    // 通过内参转化为相机坐标系的坐标
    float X_camera = (center.x - cx) * depth_value / fx;
    float Y_camera = (center.y - cy) * depth_value / fy;
    float Z_camera = depth_value;

    geometry_msgs::msg::PointStamped point_in_camera;
    point_in_camera.header.frame_id = "camera_frame";
    point_in_camera.point.x = X_camera;
    point_in_camera.point.y = Y_camera;
    point_in_camera.point.z = Z_camera;

    /*
    // Transform the point from camera frame to map frame
    try {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("map", "camera_frame", tf2::TimePointZero);
        geometry_msgs::msg::PointStamped point_in_map;
        tf2::doTransform(point_in_camera, point_in_map, transform_stamped);


        // Create and publish the goal
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = point_in_map.point.x;
        goal.pose.position.y = point_in_map.point.y;
        goal.pose.position.z = point_in_map.point.z;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        goal_publisher_->publish(goal);

        // point_in_map就是在map下的坐标

        // Publish the transformed point
        // point_publisher_->publish(point_in_map);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
    }
    */
    
  }
}