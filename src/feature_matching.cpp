#include "feature_matching.h"

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

std::vector<MatchPoints> getMatchPoints(const std::vector<cv::KeyPoint>& keypoints1,
                                        const std::vector<cv::KeyPoint>& keypoints2,
                                        const std::vector<cv::DMatch>& matches) {
  std::vector<MatchPoints> matchPoints{};
  for (const auto& match : matches) {
    Eigen::Vector2d pt1(keypoints1[match.queryIdx].pt.x, keypoints1[match.queryIdx].pt.y);
    Eigen::Vector2d pt2(keypoints2[match.trainIdx].pt.x, keypoints2[match.trainIdx].pt.y);
    matchPoints.push_back({pt1, pt2});
  }
  return matchPoints;
}

std::vector<MatchPoints> zipPoints(const std::vector<Eigen::Vector2d>& points1,
                                   const std::vector<Eigen::Vector2d>& points2) {
  if (points1.size() != points2.size()) {
    throw std::invalid_argument("points1.size() must match points2.size()");
  }
  std::vector<MatchPoints> matchPoints{};
  for (auto i = 0; i < points1.size(); ++i) {
    matchPoints.push_back({points1[i], points2[i]});
  }
  return matchPoints;
}
