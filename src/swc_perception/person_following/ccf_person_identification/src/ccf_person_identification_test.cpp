#include <iostream>
#include <opencv2/opencv.hpp>
#include <ccf_person_identification/person_classifier.hpp>
#include <chrono>
using namespace std;
using namespace std::chrono;

using namespace ccf_person_classifier;




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ccf_person_identification_test");

    std::unique_ptr<BodyClassifier> classifier(new BodyClassifier(node));

    std::string package_path = ament_index_cpp::get_package_share_directory("ccf_person_identification");

    // std::string package_prefix = ament_index_cpp::get_package_prefix("ccf_person_identification");

    std::cout << "package_path: " << package_path << std::endl;

    // std::string package_path = ament_index_cpp::get_package_prefix("ccf_person_identification");
    std::string dataset_dir = package_path + "/data/test";
    


    // To cacular the fps
    static double fps = 0.0;
    static int frameCount = 0;
    static auto lastTime = system_clock::now();
    static auto curTime = system_clock::now();

	
    auto duration = duration_cast<microseconds>(curTime - lastTime);
    double duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;

    // fps = frameCount / duration_s;
    // int height = src.rows;//row表示行，rows表示行的总数，即图像的高
    // int width = src.cols;//col表示列，cols表示列的总数，即图像的宽


    // Train the classifier with the first ten frames and test it with the remaining frames
    for (int i = 1; i <= 14; i++) {
        std::unordered_map<std::string, cv::Mat> pos, neg1, neg2;

        pos["body"] = cv::imread((boost::format("%s/p%02d.jpg") % dataset_dir % i).str());
        neg1["body"] = cv::imread((boost::format("%s/n%02d-01.jpg") % dataset_dir % i).str());
        neg2["body"] = cv::imread((boost::format("%s/n%02d-02.jpg") % dataset_dir % i).str());
	
        cv::Mat img;
        int height = pos["body"].rows;
        int width = pos["body"].cols;
        cout << "origin size() : height: " << pos["body"].rows << "    width: " << pos["body"].cols << endl;
        cv::resize(pos["body"], img, cv::Size(640, 480));
        cout << "img.size() : height: " << img.rows << "    width: " << img.cols << endl;	
        pos["body"] = img;

        if (!pos["body"].data || !neg1["body"].data || !neg2["body"].data) {
            std::cerr << "error: failed to open image!! image_id: " << i << std::endl;
            return 1;
        }

        if (i <= 10) {
            RCLCPP_INFO(node->get_logger(), "training");
        } else {
            RCLCPP_INFO(node->get_logger(), "testing");
        }

        cv::Mat pos_result;
        cv::resize(pos["body"], pos_result, cv::Size(128, 256));

        cv::Mat neg1_result;
        cv::resize(neg1["body"], neg1_result, cv::Size(128, 256));

        cv::Mat neg2_result;
        cv::resize(neg2["body"], neg2_result, cv::Size(128, 256));

        Input::Ptr input(new PersonInput());
        Features::Ptr features(new PersonFeatures());

        classifier->extractInput(input, pos);
        classifier->extractFeatures(features, input);
        cv::Scalar pos_color = *classifier->predict(features) > 0.0 ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::rectangle(pos_result, cv::Point(0, 0), cv::Point(128, 256), pos_color, 5);

        RCLCPP_INFO(node->get_logger(), "pos: %f", *classifier->predict(features));

        classifier->extractInput(input, neg1);
        classifier->extractFeatures(features, input);
        cv::Scalar neg1_color = *classifier->predict(features) > 0.0 ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::rectangle(neg1_result, cv::Point(0, 0), cv::Point(128, 256), neg1_color, 5);

        RCLCPP_INFO(node->get_logger(), "neg1: %f", *classifier->predict(features));

        classifier->extractInput(input, neg2);
        classifier->extractFeatures(features, input);
        cv::Scalar neg2_color = *classifier->predict(features) > 0.0 ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::rectangle(neg2_result, cv::Point(0, 0), cv::Point(128, 256), neg2_color, 5);

        RCLCPP_INFO(node->get_logger(), "neg2: %f", *classifier->predict(features));

        std::vector<cv::Mat> results = { pos_result, neg1_result, neg2_result };
        cv::Mat canvas;
        cv::hconcat(results, canvas);
        cv::putText(canvas, i <= 10 ? "training" : "testing", cv::Point(10, 25), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 2);
        cv::putText(canvas, i <= 10 ? "training" : "testing", cv::Point(10, 25), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(255));
	std::string result_path = (boost::format("%s/data/results/result%02d.jpg") % package_path % i).str();
        cv::imwrite(result_path, canvas);
	    cv::imshow("results", canvas);

        cv::Mat feature_map = classifier->visualize();
        if (feature_map.data) {
	    std::string feature_path = (boost::format("%s/data/features/feature%02d.jpg") % package_path % i).str();
            cv::imwrite(feature_path, feature_map);
            cv::imshow("feature_map", feature_map);
        }
        cv::waitKey(0);
	double pre;
        if (i <= 10) {
	    lastTime = system_clock::now();
            classifier->extractInput(input, pos);
            classifier->extractFeatures(features, input);
            classifier->update(1.0, features);
	    pre = *classifier->predict(features);
	    curTime = system_clock::now();
	    duration = duration_cast<microseconds>(curTime - lastTime);
	    // cout << "duration:  " << duration << endl;
	    duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;
	    fps = 1 / duration_s;
	    cout << "duration_s: " << duration_s << endl;
	    cout << "fps:  " << fps << endl;

	    lastTime = system_clock::now();
            classifier->extractInput(input, neg1);
            classifier->extractFeatures(features, input);
            classifier->update(-1.0, features);
	    pre = *classifier->predict(features);
    	    curTime = system_clock::now();
	    duration = duration_cast<microseconds>(curTime - lastTime);
	    duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;
	    fps = 1 / duration_s;
	    cout << "duration_s: " << duration_s << endl;
	    cout << "fps:  " << fps << endl;	    


	    lastTime = system_clock::now();
            classifier->extractInput(input, neg2);
            classifier->extractFeatures(features, input);
            classifier->update(-1.0, features);
	    pre = *classifier->predict(features);
	    curTime = system_clock::now();
            duration = duration_cast<microseconds>(curTime - lastTime);
	    // cout << "duration:  " << duration << endl;
            duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;
	    cout << "duration_s: " << duration_s << endl;
	    fps = 1 / duration_s;
            cout << "fps:  " << fps << endl;
        }
    }


/*
     // To cacular the fps
 23     static double fps = 0.0;
 24     static int frameCount = 0;
 25     static auto lastTime = system_clock::now();
 26     static auto curTime = system_clock::now();
 27
 28
 29     auto duration = duration_cast<microseconds>(curTime - lastTime);
 30     double duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;
 31
 32     // fps = frameCount / duration_s;

 */
    rclcpp::shutdown();
    return 0;
}
