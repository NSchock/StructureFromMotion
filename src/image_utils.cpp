#include "image_utils.h"

#include <filesystem>
#include <opencv2/opencv.hpp>

bool isSupportedFileType(const std::filesystem::path &path, const std::vector<std::string> &extensions) {
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return std::find(extensions.begin(), extensions.end(), extension) != extensions.end();
}

std::tuple<std::vector<cv::Mat>, std::vector<std::filesystem::path>> loadImages(
    const std::filesystem::path &path, const std::vector<std::string> &extensions, int imColor) {
  std::vector<cv::Mat> images;
  std::vector<std::filesystem::path> names;

  for (const std::filesystem::directory_entry entry : std::filesystem::directory_iterator(path)) {
    if (std::filesystem::is_regular_file(entry) && isSupportedFileType(entry.path(), extensions)) {
      cv::Mat img = cv::imread(entry.path().string(), imColor);
      if (img.data != nullptr) {
        images.emplace_back(std::move(img));
        names.emplace_back(entry.path().stem());
      }
    }
  }
  return {images, names};
}

std::string readDirectory() {
  std::cout << "Enter a directory: ";
  std::string dirName;
  std::getline(std::cin, dirName);
  return dirName;
}

std::string readExtensions() {
  std::cout << "Enter admissible extensions (comma separated list): ";
  std::string extsString;
  std::getline(std::cin, extsString);
  return extsString;
}

std::vector<std::string> parseExtensions(std::string extsString) {
  // TODO: this is probably not great as it changes extsString
  // but it does not change the og, as we are passing a copy
  // should use string_view instead
  std::replace(extsString.begin(), extsString.end(), ',', ' ');

  std::stringstream extsStream(extsString);
  std::vector<std::string> extensions;

  for (std::string ext; extsStream >> ext;) {
    extensions.push_back(ext);
    if (extsStream.peek() == ' ') {
      extsStream.ignore();
    }
  }
  return extensions;
}
