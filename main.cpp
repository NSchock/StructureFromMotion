#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "image_utils.h"

int main() {
  // Load images
  std::string directory{readDirectory()};
  std::vector<std::string> extensions{parseExtensions(readExtensions())};
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

  cv::Mat imgMatches;
  cv::drawMatches(images[0], keypoints[0], images[1], keypoints[1], matches, imgMatches);
  cv::imshow("Matches", imgMatches);
  cv::waitKey(0);

  //for (cv::Mat img : images) {
  //  cv::imshow("Displaying image", img);
  //  int key = cv::waitKey(0);
  //  if (key == 's') {
  //    continue;
  //  }
  //}

  return 0;
}