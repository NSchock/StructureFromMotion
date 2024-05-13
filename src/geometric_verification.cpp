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
  float numToTest{std::numeric_limits<float>::infinity()};

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

std::vector<Eigen::Matrix3f> generateFMatrix(const std::vector<MatchPoints>& sampleMatchPoints, bool normalize) {
  return std::vector<Eigen::Matrix3f>();
}

std::vector<Eigen::Matrix3f> generateFMatrix7Points(const std::vector<MatchPoints>& sampleMatchPoints, bool normalize) {
  Eigen::Matrix<float, 7, 9> eqnMatrix{constructEqnMatrix(sampleMatchPoints)};
  return std::vector<Eigen::Matrix3f>();
}

std::vector<Eigen::Matrix3f> generateFMatrix8Points(const std::vector<MatchPoints>& sampleMatchPoints, bool normalize) {
  Eigen::Matrix<float, 8, 9> eqnMatrix{constructEqnMatrix(sampleMatchPoints)};
  return std::vector<Eigen::Matrix3f>();
}

Eigen::Matrix<float, Eigen::Dynamic, 9> constructEqnMatrix(const std::vector<MatchPoints>& matchPoints) {
  Eigen::Matrix<float, Eigen::Dynamic, 9> eqnMatrix;
  eqnMatrix.resize(matchPoints.size(), Eigen::NoChange);
  for (auto i = 0; i < matchPoints.size(); ++i) {
    Eigen::Vector2d pt1{matchPoints[i].point1};
    Eigen::Vector2d pt2{matchPoints[i].point2};
    Eigen::Matrix<float, 1, 9> row{pt2[0] * pt1[0], pt2[0] * pt1[1], pt2[0], pt2[1] * pt1[0], pt2[1] * pt1[1], pt2[1],
                                   pt1[0],          pt1[1],          1};
    eqnMatrix.row(i) = row;
  }
  return eqnMatrix;
}