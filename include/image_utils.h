#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include <filesystem>
#include <opencv2/opencv.hpp>

bool isSupportedFileType(const std::filesystem::path &pathToFile, const std::vector<std::string> &extensions);

std::tuple<std::vector<cv::Mat>, std::vector<std::filesystem::path>> loadImages(
    const std::filesystem::path &pathToFile, const std::vector<std::string> &extensions, int imColor);

std::string readDirectory();

std::string readExtensions();

std::vector<std::string> parseExtensions(std::string extsString);

#endif