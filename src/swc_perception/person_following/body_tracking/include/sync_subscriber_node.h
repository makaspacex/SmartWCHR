#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "person_tracking_msgs/msg/persons_info.hpp"
#include "functional"
#include "rclcpp_action/rclcpp_action.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/temperature.hpp>


class SyncSubscriber : public rclcpp::Node
{
public:
  SyncSubscriber() : Node("sync_subscriber")
  {
    image_sub_.subscribe(this, "/image");
    persons_sub_.subscribe(this, "/persons_info");

    sync_ = std::make_shared<message_filters::TimeSynchronizer<
      sensor_msgs::msg::Image,
      person_tracking_msgs::msg::PersonsInfo
      >>(
        image_sub_, 
        persons_sub_, 
        10);
    

    sync_->registerCallback(&SyncSubscriber::callback, this);
  }

private:

 void callback( const sensor_msgs::msg::Image::SharedPtr image_msg,
                const person_tracking_msgs::msg::PersonsInfo::SharedPtr persons_msg)
  {
    
    RCLCPP_INFO(this->get_logger(), "Received synchronized messages:");
  }

  message_filters::Subscriber<person_tracking_msgs::msg::PersonsInfo> persons_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  

  std::shared_ptr<message_filters::TimeSynchronizer<
    sensor_msgs::msg::Image, 
    person_tracking_msgs::msg::PersonsInfo
    >> sync_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncSubscriber>());
  rclcpp::shutdown();
  return 0;
}
