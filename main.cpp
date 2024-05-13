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

int main() {
  // Load images
  // std::string directory{readDirectory()};
  std::string directory{"../images"};
  // std::vector<std::string> extensions{parseExtensions(readExtensions())};
  std::vector<std::string> extensions = {".png"};
  auto [images, names] = loadImages(directory, extensions, cv::IMREAD_GRAYSCALE);

  // Compute features
  cv::Ptr<cv::SIFT> detector = cv::SIFT::create();
  std::vector<std::vector<cv::KeyPoint>> keypoints;
  detector->detect(images, keypoints);
  std::vector<cv::Mat> descriptors;
  detector->compute(images, keypoints, descriptors);

  // Match features
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  std::vector<cv::DMatch> matches;
  // For now we're just matching features from the first two images for testing purposes. Will later adapt to match
  // across entire list of images.
  matcher->match(descriptors[0], descriptors[1], matches);
  std::vector<MatchPoints> matchPoints{getMatchPoints(keypoints[0], keypoints[1], matches)};

  Eigen::Vector2d pt1(4, 9);
  Eigen::Vector2d pt2(1, 3);
  Eigen::Vector2d pt3(2, 7);
  std::vector<Eigen::Vector2d> pts{pt1, pt2, pt3};

  //// testing normalization
  auto [normalizedPoints, transformation] = normalizePoints(pts);

  cv::Point2f ptCV(1,2);
  Eigen::Vector2d pt(ptCV.x, ptCV.y);
  std::cout << pt << "\n";


  // Testing constructEqnMatrix function
  // std::cout << matchPoints.size() << "\n";

  // std::vector<MatchPoints> sampleMatchPoints;
  // std::sample(matchPoints.begin(), matchPoints.end(), std::back_inserter(sampleMatchPoints), 7,
  //             std::mt19937{std::random_device{}()});
  // for (auto matchPt : sampleMatchPoints) {
  //   std::cout << matchPt.point1 << "," << matchPt.point2 << "\n";
  // }
  // auto eqnMat{constructEqnMatrix(sampleMatchPoints)};
  // std::cout << eqnMat << "\n";

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