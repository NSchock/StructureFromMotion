#ifndef KEYPOINT_DETECTION_H
#define KEYPOINT_DETECTION_H

#include <opencv2/opencv.hpp>

#include "keypoint.h"

class KeypointDetector {
 public:
  virtual ~KeypointDetector() = default;

  /**
   * Computes the keypoints for a given image, without descriptors. Unless you want to use custom descriptors, it is
   * probably better to call computeKeypointsWithDescriptors instead.
   * @param image: The image to compute keypoints for.
   * @return The vector of keypoints (without descriptors).
   */
  virtual std::vector<Keypoint> computeKeypoints(const cv::Mat& image) const = 0;

  /**
   * Computes the descriptors for a vector of keypoints.
   * @param image: The image that the keypoints are keypoints for.
   * @param[in,out] keypoints: The list of keypoints to compute descriptors for. Modified in-place, overwriting any
   * previous descriptor values.
   */
  virtual void computeDescriptors(const cv::Mat& image, std::vector<Keypoint>& keypoints) const = 0;

  /**
   * Computes the keypoints for a given image, with descriptors.
   * @param image: The image to compute keypoints for.
   * @return The vector of keypoints (with descriptors).
   */
  virtual std::vector<Keypoint> computeKeypointsWithDescriptors(const cv::Mat& image) const = 0;
};

/**
 * A class for detecting keypoints/computing their features using SIFT.
 * @todo this uses OpenCV's SIFT implementation. Makes our own code much worse. Should implement from scratch.
 */
class SIFTDetector : public KeypointDetector {
 public:
  SIFTDetector(int numFeatures = 0, int numOctaveLayers = 3, double contrastThreshold = 0.09,
               double edgeThreshold = 10.0, double sigma = 1.6, bool enablePreciseUpscale = false)
      : m_detector{cv::SIFT::create(numFeatures, numOctaveLayers, contrastThreshold, edgeThreshold, sigma,
                                    enablePreciseUpscale)} {}
  virtual ~SIFTDetector() = default;

  std::vector<Keypoint> computeKeypoints(const cv::Mat& image) const override;

  void computeDescriptors(const cv::Mat& image, std::vector<Keypoint>& keypoints) const override;

  std::vector<Keypoint> computeKeypointsWithDescriptors(const cv::Mat& image) const override;

 private:
  int m_numFeatures{0};
  int m_numOctaveLayers{3};
  double m_contrastThreshold{0.09};
  double m_edgeThreshold{10.0};
  double m_sigma{1.6};
  bool m_enablePreciseUpscale{false};
  cv::Ptr<cv::SIFT> m_detector{cv::SIFT::create()};
};

#endif