// OpenCV2
#include <opencv2/opencv.hpp>
// Eigen3
#include <Eigen/Eigen>
// GTest
#include <gtest/gtest.h>
// Internal
#include "Converter.hpp"


TEST(toDescriptorVector, Empty) {
  cv::Mat input;
  const auto desc = ORB_SLAM2::Converter::toDescriptorVector(input);
  ASSERT_TRUE(desc.empty());
}

TEST(toDescriptorVector, OneRow) {
  cv::Mat input = (cv::Mat_<char>(1, 1) << 0);
  const auto desc = ORB_SLAM2::Converter::toDescriptorVector(input);
  ASSERT_EQ(1, desc.size());
}

TEST(toDescriptorVector, Example) {
  cv::Mat input = (cv::Mat_<char>(3, 32));
  const auto desc = ORB_SLAM2::Converter::toDescriptorVector(input);
  ASSERT_EQ(3, desc.size());
}

