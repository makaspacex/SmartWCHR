// 这个文件需要改一下
// #include <monocular_person_following/context.hpp>
#include "include/context.hpp"
// #include <cv_bridge/cv_bridge.h>
#include <ccf_person_identification/person_classifier.hpp>
#include <opencv2/opencv.hpp>



Context::Context() {            // 构造函数，重置行人分类器
    auto node = rclcpp::Node::make_shared("ccf_person_identification");
    classifier.reset(new PersonClassifier(node));

    // 这两个feature_bank的类型是循环缓冲区，定义大小
    // pos_feature_bank.resize(nh.param<int>("feature_bank_size", 32));
    // neg_feature_bank.resize(nh.param<int>("feature_bank_size", 32));
    // 暂时都写死成32
    pos_feature_bank.resize(32);
    neg_feature_bank.resize(32);


    context_publisher_ = node->create_publisher<std_msgs::msg::String>("context_topic", 10);

}

Context::~Context() {}


void Context::context_log(std::string const & text) {
    auto test_msg = std_msgs::msg::String();
    test_msg.data = text;
    context_publisher_->publish(test_msg);
}


/*
输入参数：
1、原始图片
2、Tracklet的指针
    a、获得人体检测框(cv::Rect)
    b、获取对应行人的特征，存入传入指针指向对象的input和features
疑惑，为什么以unordered_map的形式传入？
因为传入的是很多Tracklet的指针，对获得的行人检测框都获取特征
*/
void Context::extract_features(const cv::Mat& bgr_image, std::unordered_map<long unsigned int, Tracklet::Ptr>& tracks) {  
    for(auto& track: tracks) {
        if(!track.second->person_region) {
            continue;
        }

        if(track.second->person_region->width < 20 || track.second->person_region->height < 20) {
            continue;
        }

        try {
            track.second->input.reset(new PersonInput());
            track.second->features.reset(new PersonFeatures());

            std::unordered_map<std::string, cv::Mat> images;
            images["body"] = cv::Mat(bgr_image, *track.second->person_region);

            // // 创建窗口
            // cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
            
            // // 显示图片
            // cv::imshow("Display Image", images["body"]);
            
            // // 等待用户按键
            // cv::waitKey(0);

            if(!classifier->extractInput(track.second->input, images)) {
                RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "failed to extract input data");
                // RCLCPP_WARN_STREAM("failed to extract input data");
                continue;
            }

            if(!classifier->extractFeatures(track.second->features, track.second->input)) {
                RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "failed to extract input features");
                // RCLCPP_WARN_STREAM("failed to extract input features");
                continue;
            }
            
        } catch(const std::exception& e) {
            context_log("[ERROR]" + std::string(e.what()));
        }
        
        context_log("There is no error on the process of extract feature");

        
    }
}


/*
输入参数：
1、label，应该是在主程序中得到的
2、Tracklet指针
用到的Tracklet成员
track->track_msg->id  // 用于获得检测框的编号    // 做了修改，Tracklet类不再有track_msg变量，只有id这个成员变量，因为也只用到了这个
track->classifier_confidences  // 用于记录当前检测框的confidence
track->features

*/
bool Context::update_classifier(double label, const Tracklet::Ptr& track) {
    auto pred = classifier->predict(track->features, classifier_confidences[track->id]);
    if(pred) {
        track->confidence = *pred;
    }
    track->classifier_confidences = classifier_confidences[track->id];

    auto& p_bank = label > 0.0 ? pos_feature_bank : neg_feature_bank;
    auto& n_bank = label > 0.0 ? neg_feature_bank : pos_feature_bank;

    if(!n_bank.empty()) {
        size_t i = std::uniform_int_distribution<>(0, n_bank.size())(mt);
        classifier->update(-label, n_bank[i]);
    }

    if(!p_bank.full()) {
        p_bank.push_back(track->features);
    } else {
        size_t i = std::uniform_int_distribution<>(0, p_bank.size())(mt);
        p_bank[i] = track->features;
    }

    return classifier->update(label, track->features);
}

/*
需要的Tracklet的成员
track->features
track->track_msg->id
*/
boost::optional<double> Context::predict(const Tracklet::Ptr& track) {
    auto pred = classifier->predict(track->features, classifier_confidences[track->id]);
    if(pred) {
        track->confidence = *pred;
    }
    track->classifier_confidences = classifier_confidences[track->id];

    return pred;
}

std::vector<std::string> Context::classifier_names() const {
    return classifier->classifierNames();
}


cv::Mat Context::visualize_body_features() {
    ccf_person_classifier::BodyClassifier::Ptr body_classifier = classifier->getClassifier<ccf_person_classifier::BodyClassifier>("body");
    if(body_classifier) {
        cv::Mat feature_map = body_classifier->visualize();
        return feature_map;
    }

    return cv::Mat();
}

