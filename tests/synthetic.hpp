// Shared fixtures for the ORB-SLAM2 unit tests.
//
// Everything here exists because three or more test files need it. The two things
// worth knowing before using it:
//
//   * Frame/KeyFrame/MapPoint keep mutable statics (Frame::fx.., Frame::mbInitialComputations,
//     nNextId). Every test must therefore use the SAME calibration -- Calib() below -- and no
//     test may assume ids start at zero.
//   * ORB-SLAM2 raw-news KeyFrames and MapPoints and never frees them. Tests leak on purpose;
//     do not wrap them in smart pointers, the library stores the raw pointers itself.

#pragma once

#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "orbslam/core/Converter.hpp"
#include "orbslam/core/Frame.hpp"
#include "orbslam/core/KeyFrame.hpp"
#include "orbslam/features/KeyFrameDatabase.hpp"
#include "orbslam/core/Map.hpp"
#include "orbslam/core/MapPoint.hpp"
#include "orbslam/mapping/LocalMapping.hpp"
#include "orbslam/loop_closing/LoopClosing.hpp"
#include "orbslam/features/ORBextractor.hpp"
#include "orbslam/features/ORBVocabulary.hpp"

namespace synth {

// Calibration used by every test. TUM1-ish intrinsics with no distortion, and a baseline
// chosen so the synthetic plane at PlaneDepth() sits at an integer disparity.
struct Calibration {
  cv::Mat K;
  cv::Mat DistCoef;
  float bf;
  float thDepth;
};

inline float PlaneDepth() {
  return 2.0f;
}

inline Calibration& Calib() {
  static Calibration c = [] {
    Calibration t;
    t.K = cv::Mat::eye(3, 3, CV_32F);
    t.K.at<float>(0, 0) = 517.306408f;
    t.K.at<float>(1, 1) = 516.469215f;
    t.K.at<float>(0, 2) = 318.643040f;
    t.K.at<float>(1, 2) = 255.313989f;
    t.DistCoef = cv::Mat::zeros(4, 1, CV_32F);
    t.bf = 40.0f;
    t.thDepth = 40.0f * 40.0f / 517.306408f;    // bf * ThDepth / fx, as Tracking computes it
    return t;
  }();
  return c;
}

// Disparity of the synthetic plane, in pixels. Integer by construction (40 / 2 = 20).
inline int PlaneDisparity() {
  return static_cast<int>(Calib().bf / PlaneDepth());
}

inline int Width() {
  return 640;
}
inline int Height() {
  return 480;
}
inline int Margin() {
  return 120;
}

// A plane of random 3x3 dots on a mid-grey ground, large enough to pan a WidthxHeight window
// over. Same construction as tools/make_synth_seq.py, so the tests and the smoke test look at
// the same kind of image.
inline const cv::Mat& Canvas() {
  static cv::Mat canvas = [] {
    const int cw = Width() + 2 * Margin(), ch = Height() + 2 * Margin();
    cv::Mat c(ch, cw, CV_8U, cv::Scalar(128));
    cv::RNG rng(7);
    for(int i = 0; i < 60000; ++i) {
      const int x = rng.uniform(2, cw - 2), y = rng.uniform(2, ch - 2);
      c(cv::Rect(x - 1, y - 1, 3, 3)).setTo(rng.uniform(0, 2) ? 255 : 0);
    }
    return c;
  }();
  return canvas;
}

// The window at horizontal offset `shift`, i.e. the camera panned `shift` pixels right.
inline cv::Mat SynthImage(int shift = 0) {
  return Canvas()(cv::Rect(Margin() + shift, Margin(), Width(), Height())).clone();
}

// The right image of a stereo pair whose left image is SynthImage(shift).
inline cv::Mat SynthImageRight(int shift = 0) {
  return SynthImage(shift + PlaneDisparity());
}

// Metric depth map for SynthImage(): a fronto-parallel plane, already in metres (Tracking
// applies DepthMapFactor before it reaches Frame, so Frame always sees metres).
inline cv::Mat SynthDepth() {
  return cv::Mat(Height(), Width(), CV_32F, cv::Scalar(PlaneDepth()));
}

// An image with no corners at all, for "extractor finds nothing" tests.
inline cv::Mat BlankImage() {
  return cv::Mat(Height(), Width(), CV_8U, cv::Scalar(128));
}

inline ORB_SLAM2::ORBextractor* Extractor() {
  static ORB_SLAM2::ORBextractor e(1000, 1.2f, 8, 20, 7);
  return &e;
}

inline ORB_SLAM2::ORBextractor* ExtractorRight() {
  static ORB_SLAM2::ORBextractor e(1000, 1.2f, 8, 20, 7);
  return &e;
}

// A small vocabulary trained on real ORB descriptors from the synthetic images. This is what
// lets the tests exercise ComputeBoW, KeyFrameDatabase, LoopClosing and System without loading
// the 100MB Vocabulary/ORBvoc.txt.
//
// The depth is not free to choose: Frame::ComputeBoW and KeyFrame::ComputeBoW both call
// transform(..., 4), i.e. they index nodes four levels up from the leaves, so a vocabulary
// with fewer than five levels yields an empty BowVector. 3^6 = 729 words.
inline ORB_SLAM2::ORBVocabulary& TinyVocab() {
  static ORB_SLAM2::ORBVocabulary voc = [] {
    std::vector<std::vector<cv::Mat>> features;
    for(int shift : {0, 10, 20, 30, 40}) {
      std::vector<cv::KeyPoint> kps;
      cv::Mat desc;
      (*Extractor())(SynthImage(shift), cv::Mat(), kps, desc);
      features.push_back(ORB_SLAM2::Converter::toDescriptorVector(desc));
    }
    ORB_SLAM2::ORBVocabulary v(3, 6, DBoW2::TF_IDF, DBoW2::L1_NORM);
    v.create(features);
    return v;
  }();
  return voc;
}

// Bounded wait on a condition another thread will satisfy. Returns false on timeout, so a
// deadlock fails the test instead of hanging it.
template <typename Predicate>
bool WaitFor(Predicate pred, std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while(std::chrono::steady_clock::now() < deadline) {
    if(pred())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

inline std::string TempPath(const std::string& name) {
  return "/tmp/orbslam2_tests_" + name;
}

// TinyVocab() written out in DBoW2 text format, for System's file-taking constructor.
inline std::string TinyVocabFile() {
  static std::string path = [] {
    const std::string p = TempPath("tiny_voc.txt");
    TinyVocab().saveToTextFile(p);
    return p;
  }();
  return path;
}

// A settings file matching Calib(), for Tracking's and System's constructors.
inline std::string SettingsYaml() {
  static std::string path = [] {
    const std::string p = TempPath("settings.yaml");
    std::ofstream f(p);
    f << "%YAML:1.0\n"
      << "Camera.fx: " << Calib().K.at<float>(0, 0) << "\n"
      << "Camera.fy: " << Calib().K.at<float>(1, 1) << "\n"
      << "Camera.cx: " << Calib().K.at<float>(0, 2) << "\n"
      << "Camera.cy: " << Calib().K.at<float>(1, 2) << "\n"
      << "Camera.k1: 0.0\nCamera.k2: 0.0\nCamera.p1: 0.0\nCamera.p2: 0.0\nCamera.k3: 0.0\n"
      << "Camera.fps: 30.0\n"
      << "Camera.RGB: 1\n"
      << "Camera.bf: " << Calib().bf << "\n"
      << "ThDepth: 40.0\n"
      << "DepthMapFactor: 1.0\n"
      << "ORBextractor.nFeatures: 1000\n"
      << "ORBextractor.scaleFactor: 1.2\n"
      << "ORBextractor.nLevels: 8\n"
      << "ORBextractor.iniThFAST: 20\n"
      << "ORBextractor.minThFAST: 7\n";
    return p;
  }();
  return path;
}

enum Sensor { kMono, kStereo, kRgbd };

inline ORB_SLAM2::Frame MakeFrame(Sensor sensor, int shift = 0, double timestamp = 0.0) {
  cv::Mat K = Calib().K.clone(), D = Calib().DistCoef.clone();
  switch(sensor) {
  case kStereo:
    return ORB_SLAM2::Frame(SynthImage(shift),
                            SynthImageRight(shift),
                            timestamp,
                            Extractor(),
                            ExtractorRight(),
                            &TinyVocab(),
                            K,
                            D,
                            Calib().bf,
                            Calib().thDepth);
  case kRgbd:
    return ORB_SLAM2::Frame(
        SynthImage(shift), SynthDepth(), timestamp, Extractor(), &TinyVocab(), K, D, Calib().bf, Calib().thDepth);
  case kMono:
  default:
    return ORB_SLAM2::Frame(SynthImage(shift), timestamp, Extractor(), &TinyVocab(), K, D, Calib().bf, Calib().thDepth);
  }
}

// A KeyFrame at the identity pose. Leaks by design (see the file header).
inline ORB_SLAM2::KeyFrame* MakeKeyFrame(ORB_SLAM2::Frame& F, ORB_SLAM2::Map* pMap, ORB_SLAM2::KeyFrameDatabase* pKFDB = nullptr) {
  F.SetPose(cv::Mat::eye(4, 4, CV_32F));
  ORB_SLAM2::KeyFrame* pKF = new ORB_SLAM2::KeyFrame(F, pMap, pKFDB);
  pKF->ComputeBoW();
  return pKF;
}

// LocalMapping::Run dereferences its loop closer unconditionally, so a running local mapper
// always needs a LoopClosing next to it. This is the pair System builds, minus Tracking.
struct Pipeline {
  ORB_SLAM2::Map map;
  ORB_SLAM2::KeyFrameDatabase kfdb{TinyVocab()};
  ORB_SLAM2::LocalMapping mapper{&map, true};
  ORB_SLAM2::LoopClosing closer{&map, &kfdb, &TinyVocab(), false};
  std::thread mapperThread, closerThread;

  Pipeline() {
    mapper.SetLoopCloser(&closer);
    closer.SetLocalMapper(&mapper);
  }

  void Start() {
    mapperThread = std::thread(&ORB_SLAM2::LocalMapping::Run, &mapper);
    closerThread = std::thread(&ORB_SLAM2::LoopClosing::Run, &closer);
  }

  // Bounded shutdown: a thread that will not finish fails the caller's test rather than
  // hanging the suite.
  bool Shutdown() {
    bool ok = true;
    if(mapperThread.joinable()) {
      mapper.RequestFinish();
      mapper.Release();
      ok = WaitFor([this] { return mapper.isFinished(); }) && ok;
      mapperThread.join();
    }
    if(closerThread.joinable()) {
      closer.RequestFinish();
      ok = WaitFor([this] { return closer.isFinished(); }) && ok;
      closerThread.join();
    }
    return ok;
  }

  ~Pipeline() {
    Shutdown();
  }

  ORB_SLAM2::KeyFrame* NewKeyFrame(int shift = 0) {
    ORB_SLAM2::Frame F = MakeFrame(kRgbd, shift);
    ORB_SLAM2::KeyFrame* pKF = MakeKeyFrame(F, &map, &kfdb);
    map.AddKeyFrame(pKF);
    return pKF;
  }
};

// Tcw for a camera at world position (tx, ty, tz) looking down +z with no rotation.
inline cv::Mat PoseAt(float tx, float ty, float tz) {
  cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
  Tcw.at<float>(0, 3) = -tx;
  Tcw.at<float>(1, 3) = -ty;
  Tcw.at<float>(2, 3) = -tz;
  return Tcw;
}

// Project a world point with Calib() under the given world-to-camera pose.
inline cv::Point2f Project(const cv::Mat& Tcw, const cv::Mat& Pw) {
  const cv::Mat Pc = Tcw.rowRange(0, 3).colRange(0, 3) * Pw + Tcw.rowRange(0, 3).col(3);
  const float invz = 1.0f / Pc.at<float>(2);
  return cv::Point2f(Calib().K.at<float>(0, 0) * Pc.at<float>(0) * invz + Calib().K.at<float>(0, 2),
                     Calib().K.at<float>(1, 1) * Pc.at<float>(1) * invz + Calib().K.at<float>(1, 2));
}

inline cv::Mat Point3(float x, float y, float z) {
  return (cv::Mat_<float>(3, 1) << x, y, z);
}

}    // namespace synth
