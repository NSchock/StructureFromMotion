#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <random>

#include "feature_matching.h"
#include "geometric_verification.h"
#include "image_utils.h"
#include "normalization.h"
#include "keypoint.h"
#include "keypoint_detection.h"

int main() {
  // Load images
  // std::string directory{readDirectory()};
  std::string directory{"../images"};
  // std::vector<std::string> extensions{parseExtensions(readExtensions())};
  std::vector<std::string> extensions = {".png"};
  auto [images, names] = loadImages(directory, extensions, cv::IMREAD_GRAYSCALE);

  // Compute features
  SIFTDetector detector{};
  std::vector<Keypoint> keypoints{detector.computeKeypointsWithDescriptors(images[0])};

  // Match features
  //cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  //std::vector<cv::DMatch> matches;
  // For now we're just matching features from the first two images for testing purposes. Will later adapt to match
  // across entire list of images.
  //matcher->match(descriptors[0], descriptors[1], matches);
  //std::vector<MatchPoints> matchPoints{getMatchPoints(keypoints[0], keypoints[1], matches)};

  //auto [fundMat, inliers] = ransacFundMat(matchPoints);
  //std::cout << fundMat << "\n";
  //std::cout << "num inliers: " << inliers.size() << "\n";
  //std::cout << "num match points: " << matchPoints.size() << "\n";

  // std::vector<MatchPoints> sampleMatchPoints;
  // std::sample(matchPoints.begin(), matchPoints.end(), std::back_inserter(sampleMatchPoints), 7,
  //             std::mt19937{std::random_device{}()});
  // generateFMatrix7Points(sampleMatchPoints);

  // Draw matches
  // cv::Mat imgMatches;
  // cv::drawMatches(images[0], keypoints[0], images[1], keypoints[1], matches, imgMatches);
  // cv::imshow("Matches", imgMatches);
  // cv::waitKey(0);

  // Compute fundamental matrix

  // for (cv::Mat img : images) {
  //   cv::imshow("Displaying image", img);
  //   int key = cv::waitKey(0);
  //   if (key == 's') {
  //     continue;
  //   }
  // }

  return 0;
}