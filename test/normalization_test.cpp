#include "normalization.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace {

constexpr double prec{.0000001};

std::vector<Eigen::Vector2d> pts{Eigen::Vector2d(1, 2), Eigen::Vector2d(3, 4)};

Transform2d expectedTranslation{Eigen::Translation2d(-2, -3)};
std::vector<Eigen::Vector2d> expectedTranslatedPts{Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, 1)};

Transform2d expectedScalar{Eigen::Scaling(std::sqrt(2) / (0.5 * (std::sqrt(5) + std::sqrt(25))))};
std::vector<Eigen::Vector2d> expectedScaledPts{Eigen::Vector2d(0.39087901517, 0.78175803033),
                                               Eigen::Vector2d(1.17263704551, 1.56351606068)};

// Unfortunately I've chosen example points such that the translated points don't require scaling
// TODO test on new points
Transform2d expectedNormalization{expectedTranslation};
std::vector<Eigen::Vector2d> expectedNormalizedPts{Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, 1)};

TEST(translatePointsTest, translationMatrix) {
  auto [translatedPts, translation] = translatePoints(pts);
  ASSERT_TRUE(translation.isApprox(expectedTranslation, prec));
}

TEST(translatePointsTest, translatedPoints) {
  auto [translatedPts, translation] = translatePoints(pts);
  ASSERT_EQ(translatedPts.size(), expectedTranslatedPts.size());
  for (auto i = 0; i < translatedPts.size(); ++i) {
    ASSERT_TRUE(translatedPts[i].isApprox(expectedTranslatedPts[i], prec));
  }
}

TEST(scalePointsTest, scaleMatrix) {
  auto [scaledPts, scalar] = scalePoints(pts);
  ASSERT_TRUE(scalar.isApprox(expectedScalar, prec));
}

TEST(scalePointsTest, scaledPoints) {
  auto [scaledPts, scalar] = scalePoints(pts);
  ASSERT_EQ(scaledPts.size(), expectedScaledPts.size());
  for (auto i = 0; i < scaledPts.size(); ++i) {
    ASSERT_TRUE(scaledPts[i].isApprox(expectedScaledPts[i], prec));
  }
}

TEST(normalizePointsTest, normalizationMatrix) {
  auto [normalizedPts, normalization] = normalizePoints(pts);
  ASSERT_TRUE(normalization.isApprox(expectedNormalization, prec));
}

TEST(normalizePointsTest, normalizedPoints) {
  auto [normalizedPts, normalization] = normalizePoints(pts);
  ASSERT_EQ(normalizedPts.size(), expectedNormalizedPts.size());
  for (auto i = 0; i < normalizedPts.size(); ++i) {
    ASSERT_TRUE(normalizedPts[i].isApprox(expectedNormalizedPts[i], prec));
  }
}

}  // namespace