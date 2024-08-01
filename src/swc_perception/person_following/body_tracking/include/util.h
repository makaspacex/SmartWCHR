#ifndef UTIL_H_
#define UTIL_H_
#include <algorithm>
#include <tuple>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// cal the angel of lines pt1-c and pt2-c using pt1 as base
float CalAngelOfTwoVector(const cv::Point2f &c,
                          const cv::Point2f &pt1,
                          const cv::Point2f &pt2,
                          bool clock_wise = true);
#endif
