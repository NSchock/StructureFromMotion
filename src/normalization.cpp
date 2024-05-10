#include "normalization.h"

#include <Eigen/Geometry>
#include <numeric>

std::tuple<std::vector<Eigen::Vector2d>, Eigen::Transform<double, 2, Eigen::Affine>> normalizePoints(
    const std::vector<Eigen::Vector2d>& points) {
  auto [translatedPoints, translation] = translatePoints(points);
  auto [normalizedPoints, scalar] = scalePoints(translatedPoints);
  return {normalizedPoints, scalar * translation};
}

std::tuple<std::vector<Eigen::Vector2d>, Eigen::Translation2d> translatePoints(
    const std::vector<Eigen::Vector2d>& points) {
  Eigen::Vector2d sum{std::accumulate(points.begin(), points.end(), Eigen::Vector2d(0, 0))};
  Eigen::Translation2d translation(-sum[0] / points.size(), -sum[1] / points.size());
  return {applyTransformation(points, Eigen::Transform<double, 2, Eigen::Affine>(translation)), translation};
}

std::tuple<std::vector<Eigen::Vector2d>, Eigen::UniformScaling<double>> scalePoints(
    const std::vector<Eigen::Vector2d>& points) {
  std::vector<float> norms(points.size());
  std::transform(points.begin(), points.end(), norms.begin(), [](Eigen::Vector2d pt) { return pt.norm(); });
  float avgLength{std::accumulate(norms.begin(), norms.end(), 0.0f) / points.size()};
  Eigen::UniformScaling<double> scalar(std::sqrt(2) / avgLength);
  return {applyTransformation(points, Eigen::Transform<double, 2, Eigen::Affine>(scalar)), scalar};
}

std::vector<Eigen::Vector2d> applyTransformation(const std::vector<Eigen::Vector2d>& points,
                                                 const Eigen::Transform<double, 2, Eigen::Affine>& transformation) {
  std::vector<Eigen::Vector2d> transformedPoints(points.size());
  std::transform(points.begin(), points.end(), transformedPoints.begin(),
                 [transformation](Eigen::Vector2d pt) { return transformation * pt; });
  return transformedPoints;
}