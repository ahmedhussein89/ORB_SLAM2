#include "orbslam/core/Converter.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ORB_SLAM2::Converter;

TEST(Converter, SE3QuatRoundTrip) {
  cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
  Tcw.at<float>(0, 3) = 1.5f;
  Tcw.at<float>(1, 3) = -2.0f;
  Tcw.at<float>(2, 3) = 0.25f;

  const cv::Mat back = Converter::toCvMat(Converter::toSE3Quat(Tcw));

  ASSERT_EQ(back.type(), CV_32F);
  EXPECT_THAT(cv::norm(back - Tcw), ::testing::Lt(1e-5));
}

TEST(Converter, IdentityQuaternionIsUnit) {
  const std::vector<float> q = Converter::toQuaternion(cv::Mat::eye(3, 3, CV_32F));
  EXPECT_THAT(
      q,
      ::testing::ElementsAre(::testing::FloatEq(0.f), ::testing::FloatEq(0.f), ::testing::FloatEq(0.f), ::testing::FloatEq(1.f)));
}

TEST(Converter, ToDescriptorVectorSplitsRows) {
  cv::Mat desc(5, 32, CV_8U);
  cv::randu(desc, 0, 256);

  const std::vector<cv::Mat> rows = Converter::toDescriptorVector(desc);

  ASSERT_EQ(rows.size(), 5u);
  for(size_t i = 0; i < rows.size(); ++i) {
    EXPECT_EQ(rows[i].rows, 1);
    EXPECT_EQ(rows[i].cols, 32);
    EXPECT_EQ(cv::countNonZero(rows[i] != desc.row(static_cast<int>(i))), 0);
  }
}

TEST(Converter, ToCvSE3BuildsTheHomogeneousMatrix) {
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d t(1.0, 2.0, 3.0);

  const cv::Mat T = Converter::toCvSE3(R, t);

  ASSERT_EQ(T.type(), CV_32F);
  ASSERT_EQ(T.rows, 4);
  ASSERT_EQ(T.cols, 4);
  // Rotating (1,0,0) by 90 deg about z gives (0,1,0).
  EXPECT_NEAR(T.at<float>(0, 0), 0.f, 1e-6);
  EXPECT_NEAR(T.at<float>(1, 0), 1.f, 1e-6);
  EXPECT_NEAR(T.at<float>(0, 3), 1.f, 1e-6);
  EXPECT_NEAR(T.at<float>(1, 3), 2.f, 1e-6);
  EXPECT_NEAR(T.at<float>(2, 3), 3.f, 1e-6);
  EXPECT_NEAR(T.at<float>(3, 3), 1.f, 1e-6);
  EXPECT_NEAR(T.at<float>(3, 0), 0.f, 1e-6);
}

TEST(Converter, Vector3dRoundTrip) {
  const cv::Mat v = (cv::Mat_<float>(3, 1) << 1.5f, -2.5f, 0.75f);
  const cv::Mat back = Converter::toCvMat(Converter::toVector3d(v));

  ASSERT_EQ(back.type(), CV_32F);
  EXPECT_LT(cv::norm(back - v), 1e-6);
}

TEST(Converter, Vector3dFromPoint3f) {
  const Eigen::Vector3d v = Converter::toVector3d(cv::Point3f(1.f, 2.f, 3.f));
  EXPECT_NEAR(v.x(), 1.0, 1e-9);
  EXPECT_NEAR(v.y(), 2.0, 1e-9);
  EXPECT_NEAR(v.z(), 3.0, 1e-9);
}

TEST(Converter, Matrix3dRoundTrip) {
  cv::Mat R = (cv::Mat_<float>(3, 3) << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f);
  const cv::Mat back = Converter::toCvMat(Converter::toMatrix3d(R));

  ASSERT_EQ(back.type(), CV_32F);
  EXPECT_LT(cv::norm(back - R), 1e-6);
}

TEST(Converter, Sim3ToCvMatCarriesScale) {
  const g2o::Sim3 S(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0), 2.0);
  const cv::Mat T = Converter::toCvMat(S);

  // toCvMat(Sim3) folds the scale into the rotation block but leaves t alone.
  EXPECT_NEAR(T.at<float>(0, 0), 2.f, 1e-6);
  EXPECT_NEAR(T.at<float>(0, 3), 1.f, 1e-6);
}

TEST(Converter, QuaternionOfA90DegZRotation) {
  cv::Mat R = (cv::Mat_<float>(3, 3) << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f);
  const std::vector<float> q = Converter::toQuaternion(R);    // x y z w

  ASSERT_EQ(q.size(), 4u);
  EXPECT_NEAR(std::abs(q[2]), std::sqrt(0.5f), 1e-5);
  EXPECT_NEAR(std::abs(q[3]), std::sqrt(0.5f), 1e-5);
  EXPECT_NEAR(q[0], 0.f, 1e-5);
  EXPECT_NEAR(q[1], 0.f, 1e-5);
}
