#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "functional"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/temperature.hpp>
#include "person_tracking_msgs/msg/persons_info.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 包含自定义的消息规则
/*
① 对应的文件名+.hpp后缀
② 文件名大写变小写
③ 大写和小写中间的地方加‘_’
*/
#include "yolomix_msgs/msg/yolo_persons.hpp"


#include <message_filters/sync_policies/approximate_time.h>

/*
参考链接：
https://gist.github.com/zkytony/0ff81b20b9d6636ab60bb6eab46ee92c
*/

#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/synchronizer.h>

// using geometry_msgs::msg::PointStamped;
using namespace message_filters;
using namespace message_filters::sync_policies;
using namespace std::chrono_literals;

// typedef ApproximateTime<PointStamped, PointStamped> PointSyncPolicy;
// typedef Synchronizer<PointSyncPolicy> PointSync;

// typedef ApproximateTime<person_tracking_msgs::msg::PersonsInfo, sensor_msgs::msg::Image> personSyncPolicy;
// typedef Synchronizer<personSyncPolicy> PersonSync;

typedef ApproximateTime<person_tracking_msgs::msg::PersonsInfo, sensor_msgs::msg::Image, sensor_msgs::msg::LaserScan> personSyncPolicy;
typedef Synchronizer<personSyncPolicy> PersonSync;



#define UNUSED(x) (void)(x)


#ifndef MESSAGE_SYNC_NODE_H_
#define MESSAGE_SYNC_NODE_H_

using namespace std::placeholders;

// 函数指针，两个参数分别为推理结果的消息和原始图片的消息
using SmartCbType = std::function<void(
    const person_tracking_msgs::msg::PersonsInfo::ConstSharedPtr &msg, const sensor_msgs::msg::Image::ConstSharedPtr &img, const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)>;


class MessageSyncNode : public rclcpp::Node {
public:
  // 构造函数，传入node名字和回调函数指针
  MessageSyncNode(const std::string &node_name,
                     SmartCbType smart_cb = nullptr)
      : Node(node_name), smart_cb_(smart_cb) {

        smart_msg_sub_.subscribe(this, "/persons_info");
        image_sub_.subscribe(this, "/image");

        // depth_sub_.subscribe(this, "/camera/depth/image_raw");
        scan_sub_.subscribe(this, "/scan_s2_raw");
        
        // time_sync_ = std::make_shared<message_filters::TimeSynchronizer<person_tracking_msgs::msg::PersonsInfo, sensor_msgs::msg::Image>>(smart_msg_sub_, image_sub_, 10);
        
        // personSyncPolicy(200)，表示同步允许的时间间隔为200ms
        time_sync_ = std::make_shared<PersonSync>(personSyncPolicy(200),
                                            smart_msg_sub_, image_sub_, scan_sub_);


        // sync_->registerCallback(&Syncer::cb, this);
        
        
        time_sync_->registerCallback(&MessageSyncNode::syncCallback, this);

      }

private:
    void syncCallback(const person_tracking_msgs::msg::PersonsInfo::SharedPtr& smart_msg, const sensor_msgs::msg::Image::SharedPtr& image, const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg) 
    {
        
        cv::Mat cv_image = cv_bridge::toCvCopy(image, "bgr8")->image;
        auto image_size = cv_image.size();

        
        // RCLCPP_INFO(rclcpp::get_logger("MessageSyncNode"), "sync subscribe three message");
        


        // 在这里处理同步后的消息
        // 可以使用 smart_msg 和 image 进行进一步的操作
        // 20230828smart_cb_是初始化节点时，传入的函数指针
        if (smart_cb_) {
            smart_cb_(smart_msg, image, scan_msg);
        } else {
        RCLCPP_WARN(rclcpp::get_logger("MessageSync"),
            "smart_cb_ was not set");
        }

    }

    SmartCbType smart_cb_ = nullptr;
    message_filters::Subscriber<person_tracking_msgs::msg::PersonsInfo> smart_msg_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    // message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;   //订阅深度图
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;   //订阅雷达消息


    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    // std::shared_ptr<message_filters::TimeSynchronizer<person_tracking_msgs::msg::PersonsInfo, sensor_msgs::msg::Image>> time_sync_;
    std::shared_ptr<PersonSync> time_sync_;

};

#endif
