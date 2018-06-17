#ifndef EDCIRCLE_H
#define EDCIRCLE_H


#include <opencv2/opencv.hpp>
namespace edcircle {
    std::vector<cv::Point> find_circle(cv::Mat image, std::vector<float>& radiuses);
}

#endif
