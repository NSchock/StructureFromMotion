#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include <opencv2/opencv.hpp>

struct MatchPoints {
  cv::Point2f point1;
  cv::Point2f point2;
};


std::vector<MatchPoints> getMatchPoints(const std::vector<cv::KeyPoint>& keypoints1,
                                        const std::vector<cv::KeyPoint>& keypoints2,
                                        const std::vector<cv::DMatch>& matches);

#endif