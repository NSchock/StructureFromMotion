#include "geometric_verification.h"

#include <Eigen/Core>
#include <limits>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <random>
#include <ranges>

Eigen::Matrix3f ransacFundMat(const std::vector<MatchPoints>& matchPoints, int sampleSize, float inlierThreshold,
                              float prob, bool normalize, std::string eightPointAlgorithm) {
  int numSamplesTested{0};
  float numToTest = std::numeric_limits<float>::infinity();

  float bestInlierProportion{0.0f};

  Eigen::Matrix3f bestFundMat;
  std::vector<MatchPoints> bestInliers;

  std::vector<MatchPoints> sampleMatchPoints;
  auto gen = std::mt19937{std::random_device{}()};

  while (numSamplesTested < numToTest) {
    sampleMatchPoints.clear();
    std::sample(matchPoints.begin(), matchPoints.end(), std::back_inserter(sampleMatchPoints), sampleSize, gen);

    std::vector<Eigen::Matrix3f> sampleFMatrices{generateFMatrix(sampleMatchPoints)};

    for (const auto& fMat : sampleFMatrices) {
      std::vector<MatchPoints> inliers{assessInliers(sampleMatchPoints, fMat)};
      float inlierProportion{static_cast<float>(inliers.size()) / matchPoints.size()};
      if (inlierProportion > bestInlierProportion) {
        bestFundMat = fMat;
        bestInliers = inliers;
      }
      numToTest = std::log(1 - prob) / std::log(1 - std::pow((1 - inlierProportion), sampleSize));
    }
    ++numSamplesTested;
  }
  return bestFundMat;
}

std::vector<MatchPoints> assessInliers(const std::vector<MatchPoints>& matchPoints, Eigen::Matrix3f fMat) {
  return std::vector<MatchPoints>();
}

std::vector<Eigen::Matrix3f> generateFMatrix(const std::vector<MatchPoints>& sampleMatchPoints) {
  return std::vector<Eigen::Matrix3f>();
}

std::vector<Eigen::Matrix3f> generateFMatrix7Points(const std::vector<MatchPoints>& sampleMatchPoints) {
  return std::vector<Eigen::Matrix3f>();
}

std::vector<Eigen::Matrix3f> generateFMatrix8Points(const std::vector<MatchPoints>& sampleMatchPoints) {
  return std::vector<Eigen::Matrix3f>();
}

Eigen::Matrix<float, Eigen::Dynamic, 9> constructEqnMatrix(const std::vector<MatchPoints>& sampleMatchPoints) {
  Eigen::Matrix<float, Eigen::Dynamic, 9> eqnMatrix;
  eqnMatrix.resize(sampleMatchPoints.size(), Eigen::NoChange);
  for (int i = 0; i < sampleMatchPoints.size(); ++i) {
    cv::Point2f pt1{sampleMatchPoints[i].point1};
    cv::Point2f pt2{sampleMatchPoints[i].point2};
    Eigen::Matrix<float, 1, 9> row{pt2.x * pt1.x, pt2.x * pt1.y, pt2.x, pt2.y * pt1.x, pt2.y * pt1.y, pt2.y,
                                   pt1.x,         pt1.y,         1};
    eqnMatrix.row(i) = row;
  }
  return eqnMatrix;
}