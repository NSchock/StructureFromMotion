#include "geometric_verification.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <limits>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <random>
#include <ranges>
#include <unsupported/Eigen/Polynomials>

std::tuple<Eigen::Matrix3d, std::vector<MatchPoints>> ransacFundMat(const std::vector<MatchPoints>& matchPoints,
                                                                    double inlierThreshold, double prob) {
  constexpr int sampleSize{7};
  int numSamplesTested{0};
  double numToTest{std::numeric_limits<double>::infinity()};

  double bestInlierProportion{0.0};

  Eigen::Matrix3d bestFundMat;
  std::vector<MatchPoints> bestInliers;

  std::vector<MatchPoints> sampleMatchPoints;
  auto gen = std::mt19937{std::random_device{}()};

  while (numSamplesTested < numToTest) {
    sampleMatchPoints.clear();
    std::sample(matchPoints.begin(), matchPoints.end(), std::back_inserter(sampleMatchPoints), sampleSize, gen);

    std::vector<Eigen::Matrix3d> sampleFMatrices{generateFMatrix7Points(sampleMatchPoints)};

    for (const auto& fMat : sampleFMatrices) {
      std::vector<MatchPoints> inliers{assessInliers(matchPoints, fMat, inlierThreshold)};
      double inlierProportion{static_cast<double>(inliers.size()) / matchPoints.size()};
      if (inlierProportion > bestInlierProportion) {
        bestFundMat = fMat;
        bestInliers = inliers;
      }
      numToTest = std::log(1 - prob) / std::log(1 - std::pow((1 - inlierProportion), sampleSize));
    }
    ++numSamplesTested;
  }
  std::cout << "num samples tested: " << numSamplesTested << "\n";
  return {bestFundMat, bestInliers};
}

std::vector<MatchPoints> assessInliers(const std::vector<MatchPoints>& matchPoints, Eigen::Matrix3d fMat,
                                       double threshold) {
  std::vector<MatchPoints> inliers{};
  for (const auto& matchPoint : matchPoints) {
    Eigen::Vector3d pt1proj(matchPoint.point1[0], matchPoint.point1[1], 1);
    Eigen::Vector3d pt2proj(matchPoint.point2[0], matchPoint.point2[1], 1);
    Eigen::Vector3d Fpt1{fMat * pt1proj};
    Eigen::Vector3d FTpt2{fMat.transpose() * pt2proj};
    double dist{(pt2proj.transpose() * FTpt2 * pt1proj)(0) /
                (std::pow(Fpt1[0], 2) + std::pow(Fpt1[1], 2) + std::pow(FTpt2[0], 2) + std::pow(FTpt2[1], 2))};
    if (dist < threshold) {
      inliers.push_back(matchPoint);
    }
  }
  return inliers;
}

std::vector<Eigen::Matrix3d> generateFMatrix7Points(const std::vector<MatchPoints>& matchPoints, bool normalize) {
  if (matchPoints.size() != 7) {
    throw std::invalid_argument("matchPoints must have size 7.");
  }
  Eigen::MatrixXd eqnMatrix{constructEqnMatrix(matchPoints)};

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(eqnMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d F1{svd.matrixV().col(5).reshaped(3, 3).transpose()};
  Eigen::Matrix3d F2{svd.matrixV().col(6).reshaped(3, 3).transpose()};

  std::vector<Eigen::Matrix3d> fMatrices{};
  if ((F1 - F2).determinant() == 0) {
    fMatrices.push_back(F1 - F2);
  } else {
    // coeffs(i) is the coefficient of x^i in det(x*F1 + (1-x)*F2)
    // The values are just hardcoded in but I strongly dislike this, would prefer to actually compute coefficients
    // This seems to require symbolic computation though....
    Eigen::Vector4d coeffs;
    coeffs(0) = F2(0, 0) * F2(1, 1) * F2(2, 2) - F2(0, 0) * F2(1, 2) * F2(2, 1) - F2(0, 1) * F2(1, 0) * F2(2, 2) +
                F2(0, 1) * F2(1, 2) * F2(2, 0) + F2(0, 2) * F2(1, 0) * F2(2, 1) - F2(0, 2) * F2(1, 1) * F2(2, 0);
    coeffs(1) = F1(0, 0) * F2(1, 1) * F2(2, 2) - F1(0, 0) * F2(1, 2) * F2(2, 1) - F1(0, 1) * F2(1, 0) * F2(2, 2) +
                F1(0, 1) * F2(1, 2) * F2(2, 0) + F1(0, 2) * F2(1, 0) * F2(2, 1) - F1(0, 2) * F2(1, 1) * F2(2, 0) -
                F1(1, 0) * F2(0, 1) * F2(2, 2) + F1(1, 0) * F2(0, 2) * F2(2, 1) + F1(1, 1) * F2(0, 0) * F2(2, 2) -
                F1(1, 1) * F2(0, 2) * F2(2, 0) - F1(1, 2) * F2(0, 0) * F2(2, 1) + F1(1, 2) * F2(0, 1) * F2(2, 0) +
                F1(2, 0) * F2(0, 1) * F2(1, 2) - F1(2, 0) * F2(0, 2) * F2(1, 1) - F1(2, 1) * F2(0, 0) * F2(1, 2) +
                F1(2, 1) * F2(0, 2) * F2(1, 0) + F1(2, 2) * F2(0, 0) * F2(1, 1) - F1(2, 2) * F2(0, 1) * F2(1, 0) -
                3 * F2(0, 0) * F2(1, 1) * F2(2, 2) + 3 * F2(0, 0) * F2(1, 2) * F2(2, 1) +
                3 * F2(0, 1) * F2(1, 0) * F2(2, 2) - 3 * F2(0, 1) * F2(1, 2) * F2(2, 0) -
                3 * F2(0, 2) * F2(1, 0) * F2(2, 1) + 3 * F2(0, 2) * F2(1, 1) * F2(2, 0);
    coeffs(2) =
        F1(0, 0) * F1(1, 1) * F2(2, 2) - F1(0, 0) * F1(1, 2) * F2(2, 1) - F1(0, 0) * F1(2, 1) * F2(1, 2) +
        F1(0, 0) * F1(2, 2) * F2(1, 1) - 2 * F1(0, 0) * F2(1, 1) * F2(2, 2) + 2 * F1(0, 0) * F2(1, 2) * F2(2, 1) -
        F1(0, 1) * F1(1, 0) * F2(2, 2) + F1(0, 1) * F1(1, 2) * F2(2, 0) + F1(0, 1) * F1(2, 0) * F2(1, 2) -
        F1(0, 1) * F1(2, 2) * F2(1, 0) + 2 * F1(0, 1) * F2(1, 0) * F2(2, 2) - 2 * F1(0, 1) * F2(1, 2) * F2(2, 0) +
        F1(0, 2) * F1(1, 0) * F2(2, 1) - F1(0, 2) * F1(1, 1) * F2(2, 0) - F1(0, 2) * F1(2, 0) * F2(1, 1) +
        F1(0, 2) * F1(2, 1) * F2(1, 0) - 2 * F1(0, 2) * F2(1, 0) * F2(2, 1) + 2 * F1(0, 2) * F2(1, 1) * F2(2, 0) +
        F1(1, 0) * F1(2, 1) * F2(0, 2) - F1(1, 0) * F1(2, 2) * F2(0, 1) + 2 * F1(1, 0) * F2(0, 1) * F2(2, 2) -
        2 * F1(1, 0) * F2(0, 2) * F2(2, 1) - F1(1, 1) * F1(2, 0) * F2(0, 2) + F1(1, 1) * F1(2, 2) * F2(0, 0) -
        2 * F1(1, 1) * F2(0, 0) * F2(2, 2) + 2 * F1(1, 1) * F2(0, 2) * F2(2, 0) + F1(1, 2) * F1(2, 0) * F2(0, 1) -
        F1(1, 2) * F1(2, 1) * F2(0, 0) + 2 * F1(1, 2) * F2(0, 0) * F2(2, 1) - 2 * F1(1, 2) * F2(0, 1) * F2(2, 0) -
        2 * F1(2, 0) * F2(0, 1) * F2(1, 2) + 2 * F1(2, 0) * F2(0, 2) * F2(1, 1) + 2 * F1(2, 1) * F2(0, 0) * F2(1, 2) -
        2 * F1(2, 1) * F2(0, 2) * F2(1, 0) - 2 * F1(2, 2) * F2(0, 0) * F2(1, 1) + 2 * F1(2, 2) * F2(0, 1) * F2(1, 0) +
        3 * F2(0, 0) * F2(1, 1) * F2(2, 2) - 3 * F2(0, 0) * F2(1, 2) * F2(2, 1) - 3 * F2(0, 1) * F2(1, 0) * F2(2, 2) +
        3 * F2(0, 1) * F2(1, 2) * F2(2, 0) + 3 * F2(0, 2) * F2(1, 0) * F2(2, 1) - 3 * F2(0, 2) * F2(1, 1) * F2(2, 0);
    coeffs(3) = F1(0, 0) * F1(1, 1) * F1(2, 2) - F1(0, 0) * F1(1, 1) * F2(2, 2) - F1(0, 0) * F1(1, 2) * F1(2, 1) +
                F1(0, 0) * F1(1, 2) * F2(2, 1) + F1(0, 0) * F1(2, 1) * F2(1, 2) - F1(0, 0) * F1(2, 2) * F2(1, 1) +
                F1(0, 0) * F2(1, 1) * F2(2, 2) - F1(0, 0) * F2(1, 2) * F2(2, 1) - F1(0, 1) * F1(1, 0) * F1(2, 2) +
                F1(0, 1) * F1(1, 0) * F2(2, 2) + F1(0, 1) * F1(1, 2) * F1(2, 0) - F1(0, 1) * F1(1, 2) * F2(2, 0) -
                F1(0, 1) * F1(2, 0) * F2(1, 2) + F1(0, 1) * F1(2, 2) * F2(1, 0) - F1(0, 1) * F2(1, 0) * F2(2, 2) +
                F1(0, 1) * F2(1, 2) * F2(2, 0) + F1(0, 2) * F1(1, 0) * F1(2, 1) - F1(0, 2) * F1(1, 0) * F2(2, 1) -
                F1(0, 2) * F1(1, 1) * F1(2, 0) + F1(0, 2) * F1(1, 1) * F2(2, 0) + F1(0, 2) * F1(2, 0) * F2(1, 1) -
                F1(0, 2) * F1(2, 1) * F2(1, 0) + F1(0, 2) * F2(1, 0) * F2(2, 1) - F1(0, 2) * F2(1, 1) * F2(2, 0) -
                F1(1, 0) * F1(2, 1) * F2(0, 2) + F1(1, 0) * F1(2, 2) * F2(0, 1) - F1(1, 0) * F2(0, 1) * F2(2, 2) +
                F1(1, 0) * F2(0, 2) * F2(2, 1) + F1(1, 1) * F1(2, 0) * F2(0, 2) - F1(1, 1) * F1(2, 2) * F2(0, 0) +
                F1(1, 1) * F2(0, 0) * F2(2, 2) - F1(1, 1) * F2(0, 2) * F2(2, 0) - F1(1, 2) * F1(2, 0) * F2(0, 1) +
                F1(1, 2) * F1(2, 1) * F2(0, 0) - F1(1, 2) * F2(0, 0) * F2(2, 1) + F1(1, 2) * F2(0, 1) * F2(2, 0) +
                F1(2, 0) * F2(0, 1) * F2(1, 2) - F1(2, 0) * F2(0, 2) * F2(1, 1) - F1(2, 1) * F2(0, 0) * F2(1, 2) +
                F1(2, 1) * F2(0, 2) * F2(1, 0) + F1(2, 2) * F2(0, 0) * F2(1, 1) - F1(2, 2) * F2(0, 1) * F2(1, 0) -
                F2(0, 0) * F2(1, 1) * F2(2, 2) + F2(0, 0) * F2(1, 2) * F2(2, 1) + F2(0, 1) * F2(1, 0) * F2(2, 2) -
                F2(0, 1) * F2(1, 2) * F2(2, 0) - F2(0, 2) * F2(1, 0) * F2(2, 1) + F2(0, 2) * F2(1, 1) * F2(2, 0);
    Eigen::PolynomialSolver<double, 3> solver;
    solver.compute(coeffs);
    std::vector<double> realRoots{};
    solver.realRoots(realRoots, .00001);
    for (auto root : realRoots) {
      fMatrices.push_back(root * F1 + (1 - root) * F2);
    }
  }

  return fMatrices;
}

Eigen::Matrix<double, Eigen::Dynamic, 9> constructEqnMatrix(const std::vector<MatchPoints>& matchPoints) {
  Eigen::Matrix<double, Eigen::Dynamic, 9> eqnMatrix;
  eqnMatrix.resize(matchPoints.size(), Eigen::NoChange);
  for (auto i = 0; i < matchPoints.size(); ++i) {
    Eigen::Vector2d pt1{matchPoints[i].point1};
    Eigen::Vector2d pt2{matchPoints[i].point2};
    Eigen::Matrix<double, 1, 9> row{pt2[0] * pt1[0], pt2[0] * pt1[1], pt2[0], pt2[1] * pt1[0], pt2[1] * pt1[1], pt2[1],
                                    pt1[0],          pt1[1],          1};
    eqnMatrix.row(i) = row;
  }
  return eqnMatrix;
}