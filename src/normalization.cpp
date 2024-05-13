#include "normalization.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>

#include "feature_matching.h"

std::tuple<std::vector<Eigen::Vector2d>, Transform2d> normalizePoints(const std::vector<Eigen::Vector2d>& points) {
  auto [translatedPoints, translation] = translatePoints(points);
  auto [normalizedPoints, scalar] = scalePoints(translatedPoints);
  return {normalizedPoints, scalar * translation};
}

std::tuple<std::vector<Eigen::Vector2d>, Transform2d> translatePoints(const std::vector<Eigen::Vector2d>& points) {
  Eigen::Vector2d sum{std::accumulate(points.begin(), points.end(), Eigen::Vector2d(0, 0))};
  Transform2d translation{Eigen::Translation2d(-sum[0] / points.size(), -sum[1] / points.size())};
  return {applyTransformation(points, translation), translation};
}

std::tuple<std::vector<Eigen::Vector2d>, Transform2d> scalePoints(const std::vector<Eigen::Vector2d>& points) {
  std::vector<float> norms(points.size());
  std::transform(points.begin(), points.end(), norms.begin(), [](Eigen::Vector2d pt) { return pt.norm(); });
  float avgLength{std::accumulate(norms.begin(), norms.end(), 0.0f) / points.size()};
  Transform2d scalar{Eigen::Scaling(std::sqrt(2) / avgLength)};
  return {applyTransformation(points, scalar), scalar};
}

std::vector<Eigen::Vector2d> applyTransformation(const std::vector<Eigen::Vector2d>& points,
                                                 const Transform2d& transformation) {
  std::vector<Eigen::Vector2d> transformedPoints(points.size());
  std::transform(points.begin(), points.end(), transformedPoints.begin(),
                 [transformation](Eigen::Vector2d pt) { return transformation * pt; });
  return transformedPoints;
}

std::tuple<std::vector<MatchPoints>, Transform2d, Transform2d> normalizeMatches(std::vector<MatchPoints> matchPoints) {
  std::vector<Eigen::Vector2d> points1{};
  std::vector<Eigen::Vector2d> points2{};
  for (const auto& [point1, point2] : matchPoints) {
    points1.push_back(point1);
    points2.push_back(point2);
  }
  auto [normalizedPoints1, normalizationMatrix1] = normalizePoints(points1);
  auto [normalizedPoints2, normalizationMatrix2] = normalizePoints(points2);
  std::vector<MatchPoints> normalizedMatchPoints{zipPoints(normalizedPoints1, normalizedPoints2)};
  return {normalizedMatchPoints, normalizationMatrix1, normalizationMatrix2};
}
