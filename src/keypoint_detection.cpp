#include "keypoint_detection.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

std::vector<Keypoint> SIFTDetector::computeKeypoints(const cv::Mat& image) const {
  std::vector<cv::KeyPoint> cvKeypoints;
  m_detector->detect(image, cvKeypoints);

  std::vector<Keypoint> keypoints;
  for (const auto& cvKeypoint : cvKeypoints) {
    keypoints.push_back(Keypoint{cvKeypoint.pt.x, cvKeypoint.pt.y, cvKeypoint.size});
  }
  return keypoints;
}

void SIFTDetector::computeDescriptors(const cv::Mat& image, std::vector<Keypoint>& keypoints) const {
  std::vector<cv::KeyPoint> cvKeypoints;
  for (const auto& keypoint : keypoints) {
    cvKeypoints.push_back(cv::KeyPoint{static_cast<float>(keypoint.getX()), static_cast<float>(keypoint.getY()),
                                       static_cast<float>(keypoint.getScale())});
  }

  cv::Mat cvDescriptors;
  m_detector->compute(image, cvKeypoints, cvDescriptors);
  Eigen::MatrixXd descriptors;
  cv::cv2eigen(cvDescriptors, descriptors);

  for (auto i = 0; i < keypoints.size(); ++i) {
    keypoints[i].setDescriptor(descriptors.row(i));
  }
}

std::vector<Keypoint> SIFTDetector::computeKeypointsWithDescriptors(const cv::Mat& image) const {
  // Note in doing this we convert OpenCV keypoints to our keypoints in computeKeypoints, then back to OpenCV keypoints
  // in computeDescriptors. It would be better to avoid this, e.g., by implementing SIFT from scratch.
  std::vector<Keypoint> keypoints{computeKeypoints(image)};
  computeDescriptors(image, keypoints);
  return keypoints;
}
