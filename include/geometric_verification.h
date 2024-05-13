#ifndef GEOMETRIC_VERIFICATION_H
#define GEOMETRIC_VERIFICATION_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "feature_matching.h"

Eigen::Matrix3f ransacFundMat(const std::vector<MatchPoints>& matchPoints, int sampleSize = 7,
                              float inlierThreshold = 1.25f, float prob = 0.95f, bool normalize = true,
                              std::string eightPointAlgorithm = "normalized");

std::vector<MatchPoints> assessInliers(const std::vector<MatchPoints>& matchPoints, Eigen::Matrix3f fMat);

std::vector<Eigen::Matrix3f> generateFMatrix(const std::vector<MatchPoints>& sampleMatchPoints, bool normalize = true);

std::vector<Eigen::Matrix3f> generateFMatrix7Points(const std::vector<MatchPoints>& sampleMatchPoints, bool normalize = true);

std::vector<Eigen::Matrix3f> generateFMatrix8Points(const std::vector<MatchPoints>& sampleMatchPoints, bool normalize = true);

Eigen::Matrix<float, Eigen::Dynamic, 9> constructEqnMatrix(const std::vector<MatchPoints>& matchPoints);

#endif