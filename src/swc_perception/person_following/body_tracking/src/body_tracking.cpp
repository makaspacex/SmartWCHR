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

TrackingManager::TrackingManager() : Node("body_tracking_node") {
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
  goal_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  // cmd_vel发布者
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  


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
          // 计算/goal
          if (pure_visual_following) {
            Visual_follow(smart_frame);
          } else {
            Caculate_goal(smart_frame);
          }
          
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


// 在这个函数当中，完成四个状态转换的逻辑
// initial、training、Lost(reid)、tracking
// void TrackingManager::ProcessSmart(
//     std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
void TrackingManager::ProcessSmart(const Triplet &msg_image) {  
  // RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), track_info_.tracking_sta);
  // byl_log("At the beginning of ProcessSmart, the state is " + state_[track_info_.tracking_sta]);

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
    // RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "In processSmart     center_x:%.2f  center_y:%.2f  width:%.2f   height:%.2f", center_x, center_y, width, height);


    int x1 = center_x - width / 2.0;
    int y1 = center_y - height / 2.0;
    int x2 = center_x + width / 2.0;
    int y2 = center_y + height / 2.0;
    

    x1 = std::min(image_size.width, std::max(0, x1));
    y1 = std::min(image_size.height, std::max(0, y1));
    x2 = std::min(image_size.width, std::max(0, x2));
    y2 = std::min(image_size.height, std::max(0, y2));


    // tl和br分别代表检测框左上角和右下角的坐标
    cv::Point tl(x1, y1);
    cv::Point br(x2, y2);
  
    // RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "In function processSmart float     tl:(%d, %d)  br:(%d, %d)", tl.x, tl.y, br.x, br.y);

    // tl.x = std::min(image_size.width, std::max(0, tl.x));
    // tl.y = std::min(image_size.height, std::max(0, tl.y));
    // br.x = std::min(image_size.width, std::max(0, br.x));
    // br.y = std::min(image_size.height, std::max(0, br.y));



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
    


    bool stay_TRACKING = true;               // 指示是否还停留在TRACKING状态                   
    auto found = tracks.find(track_info_.track_id);

    if(found == tracks.end()) {                          // TRACKING状态中跟丢了目标，进入LOST状态 
      RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "lost target in TRACKING STATE");
      track_info_.tracking_sta = TrackingStatus::LOST;
      if (pure_visual_following) {
        VisualCancelMove();
      } else {
        CancelMove();
      }
      
      stay_TRACKING = false; 

    } 
    

    // 注释掉这段在跟踪过程中更新分类器的代码，一开始输入10张训练照片就不更新分类器了
    // 因为在跟随的过程中更新分类器速率太慢了
    
    else {
      boost::optional<double> pred = context->predict(found->second);   // 用分类器得到target框的置信度
      RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),"tracking , the pred = %.2f", pred);
      
      
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
      
      for (const auto &person : msg->persons) {
        if(person.id != track_info_.track_id)  continue;

        // 举起右手，取消跟随
        if ((person.keypoints[12].x > 0 && person.keypoints[16].x > 0) && (person.keypoints[12].y > person.keypoints[16].y)) {
          cancel = true;
          byl_log("cancel is true");
          if (pure_visual_following) {
            VisualCancelMove();
          } else {
            CancelMove();
          }
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
    // RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
    //           "Now is in LOST state......");

    long target_id = -1;
    for(const auto& track: tracks) {
        // byl_log("In the LOST sta, the tracks is not NULL" );
        boost::optional<double> pred = context->predict(track.second);       // 对每个行人框都用分类器进行预测
        RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),"after lost, the pred = %.2f", pred);

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


// // 增加的FeedSmart1代替FeedSmart
// void TrackingManager::FeedSmart1(const person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr &msg, const sensor_msgs::msg::Image::ConstSharedPtr &image) {
//   if (!rclcpp::ok()) return;
//   std::unique_lock<std::mutex> lg(smart_queue_mtx_);

//   std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> tmp(msg, image);
//   smart_image_queue_.push(tmp);

//   if (smart_image_queue_.size() > queue_len_limit_) {
//     RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
//                 "smart queue len exceed limit: %d",
//                 queue_len_limit_);
//     smart_image_queue_.pop();
//   }
//   smart_queue_condition_.notify_one();
//   lg.unlock();
// }

// 增加的FeedSmart2代替FeedSmart1
void TrackingManager::FeedSmart2(const person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr &msg, const sensor_msgs::msg::Image::ConstSharedPtr &image, const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg) {
  if (!rclcpp::ok()) return;
  std::unique_lock<std::mutex> lg(smart_queue_mtx_);

  Triplet tmp = {msg, image, scan_msg};
  // std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> tmp(msg, image);
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


// void TrackingManager::Visualization(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
void TrackingManager::Visualization(const Triplet &msg_image) {
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


    // RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "In function Visualization     center_x:%.2f  center_y:%.2f  width:%.2f   height:%.2f", center_x, center_y, width, height);



    // 计算检测框的左上角和右下角坐标
    int x1 = static_cast<int>(center_x - width / 2);
    int y1 = static_cast<int>(center_y - height / 2);
    int x2 = static_cast<int>(center_x + width / 2);
    int y2 = static_cast<int>(center_y + height / 2);

    // RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "In function Visualization float    x1:%.2f  y1:%.2f  x2:%.2f   y2:%.2f", center_x - width / 2, center_y - height / 2, center_x + width / 2, center_y + height / 2);
    // RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "In function Visualization int    x1:%d  y1:%d  x2:%d   y2:%d", x1, y1, x2, y2);

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


// void TrackingManager::Publish_goal(std::pair<person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr> &msg_image) {
void TrackingManager::Caculate_goal(const Triplet &msg_image) {

  cv::Mat input_image = cv_bridge::toCvCopy(msg_image.second, "bgr8")->image;
  
  auto msg = msg_image.first;  // msg是订阅到的PeronsInfo
  if (!msg || !rclcpp::ok()) {
    return;
  }

  cv::Point2f center(-1, -1); 
  float bbox_width = -1, bbox_height = -1;
  for (const auto &person : msg->persons) {
    if (person.id == track_info_.track_id) {
      center.x = person.center.x;
      center.y = person.center.y;
      bbox_width = person.width;
      bbox_height = person.height;
    }
  }

  if (center.x > 0) {
    // 获取相应坐标的深度值
    float depth_value = 0.0;


    /*
      data: [235.67877,   0.     , 186.26218,
           0.     , 236.79534, 111.24331,
           0.     ,   0.     ,   1.     ]


    */
    // 相机内参，通过相机标定获得
    float fx = 235.67877; 
    float fy = 236.79534;
    float cx = 186.26218;
    float cy = 111.24331;


    // 图像的宽高
    int image_width = input_image.cols, image_height = input_image.rows;
    // RCLCPP_INFO(this->get_logger(), "Image width: %d Image height: %d", image_width, image_height);
    // width = 320   height = 240
  
    // 水平和垂直视场角，通过内参计算
    float fov_x = 1.19;
    float fov_y = 0.94;


    // 每个像素点对应的角度值
    float angle_per_pixel_x = fov_x / image_width;

    // 计算检测框中心点的角度（行人中心相对相机的角度）
    float center_angle_x = (center.x - image_width / 2) * angle_per_pixel_x;

    // 计算检测框在图像中的角度范围
    float half_width_angle = (bbox_width / 2) * angle_per_pixel_x;

    // 计算角度范围
    float angle_min_ = center_angle_x - half_width_angle;
    float angle_max_ = center_angle_x + half_width_angle;


    RCLCPP_INFO(this->get_logger(), "angle_min: %.2f angle_max: %.2f", angle_min_, angle_max_);


    auto scan_msg = msg_image.third;   // 雷达消息


     // 获取雷达数据
    std::vector<float> distances = scan_msg->ranges;
    std::vector<float> angles(scan_msg->ranges.size());
    
    // 生成每个雷达测量点的角度
    for (size_t i = 0; i < angles.size(); ++i)
    {
        angles[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
    }

    // 创建用于存储指定角度范围内的距离值的数组
    std::vector<float> relevant_distances;

    // 遍历角度范围内的雷达数据
    for (size_t i = 0; i < distances.size(); ++i)
    {
        if (angles[i] >= angle_min_ && angles[i] <= angle_max_)
        {
            if (distances[i] > 0.0)  // 过滤掉无效距离
            {
                relevant_distances.push_back(distances[i]);
            }
        }
    }

    if (relevant_distances.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No data in the specified angle range");
        return;
    }
    else
    {
        // 求最小距离值
        auto min_distance = *std::min_element(relevant_distances.begin(), relevant_distances.end());
        RCLCPP_INFO(this->get_logger(), "Minimum distance in the angle range: %f meters", min_distance);
        depth_value = min_distance;
    }



    // 通过内参将图像中的坐标转化为相机坐标系的坐标
    float X_camera = (center.x - cx) * depth_value / fx;
    float Y_camera = (center.y - cy) * depth_value / fy;
    float Z_camera = depth_value;

    geometry_msgs::msg::PointStamped point_in_camera;
    point_in_camera.header.frame_id = "camera_frame";
    point_in_camera.point.x = X_camera;
    point_in_camera.point.y = Y_camera;
    point_in_camera.point.z = Z_camera;

    /*
      TODO: 根据目标行人与机器人的角度信息，设置跟随距离，使目标位置与行人保持一定距离
    */

      // 计算所需的偏移量
    float distance_to_keep = 1; // 目标位置与行人之间的距离为1m
    float offset_x = distance_to_keep * cos(center_angle_x);
    float offset_y = distance_to_keep * sin(center_angle_x);

    // 计算目标位置，减去偏移量，使机器人与目标行人保持一定距离
    point_in_camera.point.x -= offset_x;
    point_in_camera.point.y -= offset_y;
    
    
    // Transform the point from camera frame to map frame
    try {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("map", "astra_link", tf2::TimePointZero);
        geometry_msgs::msg::PointStamped point_in_map;
        tf2::doTransform(point_in_camera, point_in_map, transform_stamped);


        // Create and publish the goal
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = point_in_map.point.x;
        goal.pose.position.y = point_in_map.point.y;
        goal.pose.position.z = point_in_map.point.z;

        // 计算目标位置的角度
        // 提取摄像头当前在map坐标系下的角度信息
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w
        );

        // 转换为欧拉角
        tf2::Matrix3x3 m(q);
        double roll, pitch, robot_heading_angle;
        m.getRPY(roll, pitch, robot_heading_angle);
        
        // 目标角度等于行人相对摄像头的角度+摄像头在map中的角度
        float angle_in_map_frame = center_angle_x + robot_heading_angle;
        tf2::Quaternion goal_q;
        goal_q.setRPY(0, 0, angle_in_map_frame);
        goal.pose.orientation.x = goal_q.x();
        goal.pose.orientation.y = goal_q.y();
        goal.pose.orientation.z = goal_q.z();
        goal.pose.orientation.w = goal_q.w();

        auto now = rclcpp::Clock().now();// this->now();
        // 检查距离上次发布消息是否已过去1秒
        if ((now - last_publish_time_).seconds() >= 1.0) {
            // 发布消息
            goal_publisher_->publish(goal);
            RCLCPP_INFO(this->get_logger(), "Publish the goal topic!!!!!!!!!!!!!!!!!");
            
            // 更新上次发布消息的时间
            last_publish_time_ = now;
        }

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
    }
    
  }
}


// 将当前位姿作为目标发布，起到取消跟随的作用
void TrackingManager::CancelMove() {
  // 方案一，订阅/pose消息（应该是当前位姿），作为目标位姿发布
  // pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  //                   "/pose",
  //                   10,
  //                   [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  //                     goal_publisher_->publish(*msg);
  //                     RCLCPP_INFO(this->get_logger(), "Cancel Move, Received and Publish /Pose");
  //                   });
  


  // 方案二，通过tf获取base_footprint在map下的位姿，作为目标位姿发布
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  pose_stamped.header.stamp = this->get_clock()->now();

  try {
      // 获取 TF 变换
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
          "map", "base_footprint", tf2::TimePointZero);

      // 将 TF 变换转换为 PoseStamped 消息
      pose_stamped.pose.position.x = transform.transform.translation.x;
      pose_stamped.pose.position.y = transform.transform.translation.y;
      pose_stamped.pose.position.z = transform.transform.translation.z;
      pose_stamped.pose.orientation.x = transform.transform.rotation.x;
      pose_stamped.pose.orientation.y = transform.transform.rotation.y;
      pose_stamped.pose.orientation.z = transform.transform.rotation.z;
      pose_stamped.pose.orientation.w = transform.transform.rotation.w;

      // 发布消息
      goal_publisher_->publish(pose_stamped);

      RCLCPP_INFO(this->get_logger(), "Cancel Move, Published the basefoorprint pose");
  } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
  }
}



void TrackingManager::Visual_follow(const Triplet &msg_image) {
  
  cv::Mat input_image = cv_bridge::toCvCopy(msg_image.second, "bgr8")->image;
  
  auto msg = msg_image.first;  // msg是订阅到的PeronsInfo
  if (!msg || !rclcpp::ok()) {
    return;
  }

  cv::Point2f center(-1, -1); 
  float bbox_width = -1, bbox_height = -1;
  for (const auto &person : msg->persons) {
    if (person.id == track_info_.track_id) {
      center.x = person.center.x;
      center.y = person.center.y;
      bbox_width = person.width;
      bbox_height = person.height;
    }
  }

  if (center.x > 0) {
    // 获取相应坐标的深度值
    float depth_value = 0.0;


    /*
      data: [235.67877,   0.     , 186.26218,
           0.     , 236.79534, 111.24331,
           0.     ,   0.     ,   1.     ]


    */
    // 相机内参，通过相机标定获得
    float fx = 235.67877; 
    float fy = 236.79534;
    float cx = 186.26218;
    float cy = 111.24331;


    // 图像的宽高
    int image_width = input_image.cols, image_height = input_image.rows;
    // RCLCPP_INFO(this->get_logger(), "Image width: %d Image height: %d", image_width, image_height);
    // width = 320   height = 240
  
    // 水平和垂直视场角，通过内参计算
    float fov_x = 1.19;
    float fov_y = 0.94;


    // 每个像素点对应的角度值
    float angle_per_pixel_x = fov_x / image_width;

    // 计算检测框中心点的角度（行人中心相对相机的角度）
    float center_angle_x = -(center.x - image_width / 2) * angle_per_pixel_x;
    


    // 计算检测框在图像中的角度范围
    float half_width_angle = (bbox_width / 2) * angle_per_pixel_x;

    // 计算角度范围
    float angle_min_ = center_angle_x - half_width_angle;
    float angle_max_ = center_angle_x + half_width_angle;
    

    // 将角度标准化到 [0, 2π) 范围
    auto normalize_angle = [](float angle) -> float {
        while (angle < 0.0) angle += 2 * M_PI;
        while (angle >= 2 * M_PI) angle -= 2 * M_PI;
        return angle;
    };

    // 标准化角度范围
    angle_min_ = normalize_angle(angle_min_);
    angle_max_ = normalize_angle(angle_max_);


    // 定义用于检查角度是否在范围内的 lambda
    std::function<bool(float)> in_range;
    // 处理跨越 0 弧度的情况
    if (angle_min_ > angle_max_) {
        // 角度范围跨越 0 弧度
        in_range = [&](float angle) {
          return (angle >= angle_min_ || angle <= angle_max_);
        };
        // 角度范围已经处理过
    } else {
        // 角度范围没有跨越 0 弧度
        in_range = [&](float angle) {
          return (angle >= angle_min_ && angle <= angle_max_);
        };
    }



    // RCLCPP_INFO(this->get_logger(), "angle_min: %.2f angle_max: %.2f", angle_min_ / M_PI * 180, angle_max_ / M_PI * 180);


    auto scan_msg = msg_image.third;   // 雷达消息


     // 获取雷达数据
    std::vector<float> distances = scan_msg->ranges;
    std::vector<float> angles(scan_msg->ranges.size());
    
    // 生成每个雷达测量点的角度
    for (size_t i = 0; i < angles.size(); ++i)
    {
        angles[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
        
    }

    // 创建用于存储指定角度范围内的距离值的数组
    std::vector<float> relevant_distances;

    // 遍历角度范围内的雷达数据
    for (size_t i = 0; i < distances.size(); ++i)
    {
        // if (angles[i] >= angle_min_ && angles[i] <= angle_max_)
        if(in_range(angles[i]))
        {
            if (distances[i] > 0.0)  // 过滤掉无效距离
            {
                relevant_distances.push_back(distances[i]);
            }
        }
    }

    if (relevant_distances.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No data in the specified angle range");
        return;
    }
    else
    {
        // 求最小距离值
        auto min_distance = *std::min_element(relevant_distances.begin(), relevant_distances.end());
        // RCLCPP_INFO(this->get_logger(), "Minimum distance in the angle range: %f meters", min_distance);
        depth_value = min_distance;
    }



    // 通过内参将图像中的坐标转化为相机坐标系的坐标
    float X_camera = (center.x - cx) * depth_value / fx;
    float Y_camera = (center.y - cy) * depth_value / fy;
    float Z_camera = depth_value;

    geometry_msgs::msg::PointStamped point_in_camera;
    point_in_camera.header.frame_id = "camera_frame";
    point_in_camera.point.x = X_camera;
    point_in_camera.point.y = Y_camera;
    point_in_camera.point.z = Z_camera;

    // try {
      // geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("base_link", "astra_link", tf2::TimePointZero);
      // geometry_msgs::msg::PointStamped base_link_point;
      // tf2::doTransform(point_in_camera, base_link_point, transform_stamped);

      // 行人相对相机的角度
    float theta = center_angle_x - (20 / 180 * M_PI);
    // RCLCPP_INFO(this->get_logger(), "theta: %.2f", theta / M_PI * 180);


    double max_vx_ = 1.0;
    double min_vx_ = 0.4;
    double max_va_= 0.3;
    double gain_vx_ = 0.8;
    double gain_va_ = 0.3;
    double distance_ = 2;
    double angle_threshold_ = M_PI / 4;
    //float target_x = base_link_point.point.x;
    float target_x = point_in_camera.point.z;

    // 计算控制指令
    double va = std::min(max_va_, std::max(-max_va_, theta * gain_va_));
    double vx = 0.0;

    if (std::abs(theta) < angle_threshold_)
    {
      // RCLCPP_INFO(this->get_logger(), "target_x : %2f", target_x);
      vx = (target_x - distance_) * gain_vx_;
      if (vx < 0) {
        vx = 0;
      } else {
        vx = std::min(max_vx_, std::max(min_vx_, vx));
      }

    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Rotation too big, not moving forward");
    }

    // 创建并发布速度指令
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = vx;
    twist.angular.z = va;
    
    cmd_vel_pub_->publish(twist);

    // RCLCPP_INFO(this->get_logger(), "Published velocity command: vx=%.2f, va=%.2f", vx, va);
      

    // } catch (tf2::TransformException &ex) {
    //     RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
    // }
  }

}


void TrackingManager::VisualCancelMove() {
    // 创建并发布速度指令
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0;
  twist.angular.z = 0;
  
  cmd_vel_pub_->publish(twist);

  RCLCPP_INFO(this->get_logger(), "VisualCancelMove , Published zero velocity command");

}

