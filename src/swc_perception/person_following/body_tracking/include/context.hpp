#ifndef CONTEXT_HPP_
#define CONTEXT_HPP_

#include <random>
#include <unordered_map>
#include <boost/optional.hpp>
#include <boost/circular_buffer.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


// #include <ros/ros.h>
// #include <monocular_person_following/tracklet.hpp>
#include "include/tracklet.hpp"

namespace ccf_person_classifier {

class PersonInput;
class PersonFeatures;
class PersonClassifier;

}



class Context {
public:
    using PersonInput = ccf_person_classifier::PersonInput;
    using PersonFeatures = ccf_person_classifier::PersonFeatures;
    using PersonClassifier = ccf_person_classifier::PersonClassifier;


    void context_log(std::string const & text);

    // Context(ros::NodeHandle& nh);
    Context();
    ~Context();

public:
    void extract_features(const cv::Mat& bgr_image, std::unordered_map<long unsigned int, Tracklet::Ptr>& tracks);
    bool update_classifier(double label, const Tracklet::Ptr& track);
    boost::optional<double> predict(const Tracklet::Ptr& track);

    std::vector<std::string> classifier_names() const;
    cv::Mat visualize_body_features();

private:
    std::mt19937 mt;
    std::unique_ptr<PersonClassifier> classifier;
    std::unordered_map<long, std::vector<double>> classifier_confidences;

    boost::circular_buffer<std::shared_ptr<ccf_person_classifier::Features>> pos_feature_bank;
    boost::circular_buffer<std::shared_ptr<ccf_person_classifier::Features>> neg_feature_bank;


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr context_publisher_; 

};


#endif
