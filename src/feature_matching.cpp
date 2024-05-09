#include "feature_matching.h"

#include <opencv2/opencv.hpp>

std::vector<MatchPoints> getMatchPoints(const std::vector<cv::KeyPoint>& keypoints1,
                                        const std::vector<cv::KeyPoint>& keypoints2,
                                        const std::vector<cv::DMatch>& matches) {
  std::vector<MatchPoints> matchPoints{};
  for (const auto& match : matches) {
    matchPoints.push_back({keypoints1[match.queryIdx].pt, keypoints2[match.trainIdx].pt});
  }
  return matchPoints;
}
