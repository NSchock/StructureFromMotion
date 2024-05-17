#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

struct MatchPoints {
  Eigen::Vector2d point1;
  Eigen::Vector2d point2;
};

/**
 * @brief Converts a vector of OpenCV DMatch objects to a vector of MatchPoints describing the actual points in the two
 * images. Note this involves casting OpenCV Point2f objects to Eigen Vector2d objects.
 * @param keypoints1: The keypoints for the first image.
 * @param keypoints2: The keypoints for the second image.
 * @param matches: The vector of DMatches.
 * @return The vector of MatchPoints.
 */
std::vector<MatchPoints> getMatchPoints(const std::vector<cv::KeyPoint>& keypoints1,
                                        const std::vector<cv::KeyPoint>& keypoints2,
                                        const std::vector<cv::DMatch>& matches);

/**
 * @brief Zips two vectors of points (Eigen::Vector2d objects) into a vector of MatchPoints. It is required that the two
 * vectors of points have the same size.
 * @param points1: The first vector of points.
 * @param points2: The second vector of points.
 * @return The vector of MatchPoints.
 */
std::vector<MatchPoints> zipPoints(const std::vector<Eigen::Vector2d>& points1,
                                   const std::vector<Eigen::Vector2d>& points2);

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>> unzipPoints(const std::vector<MatchPoints> matchPoints);
#endif