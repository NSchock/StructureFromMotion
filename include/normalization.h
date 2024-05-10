#ifndef NORMALIZATION_H
#define NORMALIZATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief Normalize a collection of points by first translating so that their centroid is the origin, and then
 * scaling so that their average distance to the origin is sqrt(2).
 * @param points: The collection of points.
 * @return A tuple consisting of the normalized points, and the transformation matrix.
 */
std::tuple<std::vector<Eigen::Vector2d>, Eigen::Transform<double, 2, Eigen::Affine>> normalizePoints(
    const std::vector<Eigen::Vector2d>& points);

/**
 * @brief Calculate the translation of a collection of points so that their centroid is the origin.
 * @param points: The collection of points.
 * @return A tuple consisting of the translated points, and the translation matrix.
 */
std::tuple<std::vector<Eigen::Vector2d>, Eigen::Translation2d> translatePoints(
    const std::vector<Eigen::Vector2d>& points);

/**
 * @brief Calculate the scaling of a collection of points so that their average distance to the origin is sqrt(2).
 * @param points: The collection of points.
 * @return A tuple consisting of the scaled points, and the scaling matrix.
 */
std::tuple<std::vector<Eigen::Vector2d>, Eigen::UniformScaling<double>> scalePoints(
    const std::vector<Eigen::Vector2d>& points);

/**
 * @brief Applies an affine transformation to a vector of two-dimensional points.
 * @param points: The collection of points.
 * @param transformation: The transformation.
 * @return The transformed points.
 * 
 * @todo Would be better to replace with a more generic function.
 */
std::vector<Eigen::Vector2d> applyTransformation(const std::vector<Eigen::Vector2d>& points,
                                                 const Eigen::Transform<double, 2, Eigen::Affine>& transformation);

#endif