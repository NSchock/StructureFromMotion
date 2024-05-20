#ifndef KEYPOINT_H
#define KEYPOINT_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class Keypoint {
 public:
  Keypoint(double x, double y, double scale, Eigen::VectorXd descriptor = {})
      : Keypoint{Eigen::Vector2d{x, y}, scale, descriptor} {}

  Keypoint(Eigen::Vector2d point, double scale, Eigen::VectorXd descriptor = {})
      : m_point{point}, m_x{point[0]}, m_y{point[1]}, m_scale{scale}, m_descriptor{descriptor} {}

  Eigen::Vector2d getPoint() const { return m_point; }
  void setPoint(Eigen::Vector2d point) { m_point = point; }

  double getX() const { return m_x; }
  void setX(double x) { m_x = x; }

  double getY() const { return m_y; }
  void setY(double y) { m_y = y; }

  double getScale() const { return m_scale; }
  void setScale(double scale) { m_scale = scale; }

  /**
   * Gets the descriptor for the current keypoint.
   * Does not check whether a reasonable descriptor has been computed.
   * @return The descriptor.
   */
  Eigen::VectorXd getDescriptor() const { return m_descriptor; }
  void setDescriptor(Eigen::VectorXd descriptor) { m_descriptor = descriptor; }

 private:
  Eigen::Vector2d m_point{};
  double m_x{};
  double m_y{};
  double m_scale{};
  Eigen::VectorXd m_descriptor{};
};

#endif