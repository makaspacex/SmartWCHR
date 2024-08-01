#ifndef TRACKLET_HPP
#define TRACKLET_HPP

#include <vector>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>
#include <ccf_person_identification/online_classifier.hpp>


// 主要用于记录所有行人检测框的消息、区域、获取的特征

struct Tracklet {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<Tracklet>;

// const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg


    //Tracklet(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg)
    Tracklet(): confidence(boost::none)
	{}
       //   track_msg(msg)
    // {}

public:
    boost::optional<double> confidence;
    std::vector<double> classifier_confidences;

    boost::optional<cv::Rect> person_region;
    ccf_person_classifier::Input::Ptr input;
    ccf_person_classifier::Features::Ptr features;

    long unsigned int id;   // 这个id是用来记录对应检测框的id
    // const ai_msgs::msg::PerceptionTargets::ConstSharedPtr track_msg;   // 这个不对
};



#endif // TRACKLET_HPP
