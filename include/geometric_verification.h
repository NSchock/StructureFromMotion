#ifndef GEOMETRIC_VERIFICATION_H
#define GEOMETRIC_VERIFICATION_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "feature_matching.h"

std::tuple<Eigen::Matrix3d, std::vector<MatchPoints>> ransacFundMat(const std::vector<MatchPoints>& matchPoints,
                                                                    double inlierThreshold = 1.25, double prob = 0.95);

std::vector<MatchPoints> assessInliers(const std::vector<MatchPoints>& matchPoints, Eigen::Matrix3d fMat,
                                       double threshold = 1.25);

std::vector<Eigen::Matrix3d> generateFMatrix7Points(const std::vector<MatchPoints>& matchPoints, bool normalize = true);

Eigen::Matrix<double, Eigen::Dynamic, 9> constructEqnMatrix(const std::vector<MatchPoints>& matchPoints);

#endif