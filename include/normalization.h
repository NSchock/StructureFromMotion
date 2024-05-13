#ifndef NORMALIZATION_H
#define NORMALIZATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "feature_matching.h"

using Transform2d = Eigen::Transform<double, 2, Eigen::Affine>;

/**
 * @brief Normalize a collection of points by first translating so that their centroid is the origin, and then
 * scaling so that their average distance to the origin is sqrt(2).
 * @param points: The collection of points.
 * @return A tuple consisting of the normalized points, and the transformation matrix.
 */
std::tuple<std::vector<Eigen::Vector2d>, Transform2d> normalizePoints(const std::vector<Eigen::Vector2d>& points);

/**
 * @brief Calculate the translation of a collection of points so that their centroid is the origin.
 * @param points: The collection of points.
 * @return A tuple consisting of the translated points, and the translation matrix.
 */
std::tuple<std::vector<Eigen::Vector2d>, Transform2d> translatePoints(const std::vector<Eigen::Vector2d>& points);

/**
 * @brief Calculate the scaling of a collection of points so that their average distance to the origin is sqrt(2).
 * @param points: The collection of points.
 * @return A tuple consisting of the scaled points, and the scaling matrix.
 */
std::tuple<std::vector<Eigen::Vector2d>, Transform2d> scalePoints(const std::vector<Eigen::Vector2d>& points);

/**
 * @brief Applies an affine transformation to a vector of two-dimensional points.
 * @param points: The collection of points.
 * @param transformation: The transformation.
 * @return The transformed points.
 *
 * @todo Would be better to replace with a more generic function.
 */
std::vector<Eigen::Vector2d> applyTransformation(const std::vector<Eigen::Vector2d>& points,
                                                 const Transform2d& transformation);

/**
 * @brief Normalizes a collection of matched points between two images.
 * @param matchPoints: The collection of matched points.
 * @return A tuple consisting of the normalized matchPoints, the normalization matrix for the first image, and the
 * normalization matrix for the second image.
 */
std::tuple<std::vector<MatchPoints>, Transform2d, Transform2d> normalizeMatches(std::vector<MatchPoints> matchPoints);

#endif