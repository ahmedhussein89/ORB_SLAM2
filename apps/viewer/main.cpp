// orbslam2_viewer -- one binary, GUI by default, --headless for no window.
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "Snapshot.hpp"
#include "Viewer.hpp"
#include "orbslam/system/System.hpp"

namespace {

enum class Format { kKitti, kTum, kEuroc };

struct Options final {
  std::string vocabulary;
  std::string settings;
  std::string dataset;
  std::string association;    // TUM RGB-D
  std::string times;          // EuRoC
  std::string trajectory;
  ORB_SLAM2::System::eSensor sensor{ORB_SLAM2::System::STEREO};
  Format format{Format::kKitti};
  bool headless{false};
};

// A dataset resolved to per-frame inputs. `right` is the depth map for RGB-D.
struct Sequence final {
  std::vector<std::string> left;
  std::vector<std::string> right;
  std::vector<double> timestamps;
};

[[noreturn]] void Usage() {
  std::cerr << "Usage: orbslam2_viewer --vocabulary VOC --settings YAML --sensor mono|stereo|rgbd\n"
               "                       --format kitti|tum|euroc --dataset PATH\n"
               "                       [--assoc FILE] [--times FILE] [--headless] [--trajectory OUT]\n";
  std::exit(2);
}

std::string Need(int argc, char** argv, int& i) {
  if(++i >= argc) {
    Usage();
  }
  return argv[i];
}

Options ParseArgs(int argc, char** argv) {
  Options opt;
  for(int i = 1; i < argc; ++i) {
    const std::string arg{argv[i]};
    if(arg == "--vocabulary") {
      opt.vocabulary = Need(argc, argv, i);
    } else if(arg == "--settings") {
      opt.settings = Need(argc, argv, i);
    } else if(arg == "--dataset") {
      opt.dataset = Need(argc, argv, i);
    } else if(arg == "--assoc") {
      opt.association = Need(argc, argv, i);
    } else if(arg == "--times") {
      opt.times = Need(argc, argv, i);
    } else if(arg == "--trajectory") {
      opt.trajectory = Need(argc, argv, i);
    } else if(arg == "--headless") {
      opt.headless = true;
    } else if(arg == "--sensor") {
      const std::string value = Need(argc, argv, i);
      if(value == "mono" || value == "monocular") {
        opt.sensor = ORB_SLAM2::System::MONOCULAR;
      } else if(value == "stereo") {
        opt.sensor = ORB_SLAM2::System::STEREO;
      } else if(value == "rgbd") {
        opt.sensor = ORB_SLAM2::System::RGBD;
      } else {
        Usage();
      }
    } else if(arg == "--format") {
      const std::string value = Need(argc, argv, i);
      if(value == "kitti") {
        opt.format = Format::kKitti;
      } else if(value == "tum") {
        opt.format = Format::kTum;
      } else if(value == "euroc") {
        opt.format = Format::kEuroc;
      } else {
        Usage();
      }
    } else {
      Usage();
    }
  }
  if(opt.vocabulary.empty() || opt.settings.empty() || opt.dataset.empty()) {
    Usage();
  }
  return opt;
}

// Every loader below opens with an explicit check and reads with getline() as the
// loop condition. The examples' `while (!f.eof())` never terminates when the path
// is wrong -- the binary spins at 100% CPU with no output instead of reporting it.
std::ifstream OpenOrThrow(const std::string& path) {
  std::ifstream file{path};
  if(!file) {
    throw std::runtime_error("cannot open " + path);
  }
  return file;
}

std::string Zero6(int n) {
  std::ostringstream ss;
  ss << std::setfill('0') << std::setw(6) << n << ".png";
  return ss.str();
}

Sequence LoadKitti(const Options& opt) {
  Sequence seq;
  std::ifstream times = OpenOrThrow(opt.dataset + "/times.txt");
  std::string line;
  while(std::getline(times, line)) {
    if(line.empty()) {
      continue;
    }
    std::istringstream ss{line};
    double t = 0.0;
    ss >> t;
    seq.timestamps.push_back(t);
  }
  for(std::size_t i = 0; i < seq.timestamps.size(); ++i) {
    const std::string name = Zero6(static_cast<int>(i));
    seq.left.push_back(opt.dataset + "/image_0/" + name);
    if(opt.sensor == ORB_SLAM2::System::STEREO) {
      seq.right.push_back(opt.dataset + "/image_1/" + name);
    }
  }
  return seq;
}

Sequence LoadTum(const Options& opt) {
  Sequence seq;
  if(opt.sensor == ORB_SLAM2::System::RGBD) {
    if(opt.association.empty()) {
      throw std::runtime_error("--format tum --sensor rgbd needs --assoc <associations file>");
    }
    std::ifstream assoc = OpenOrThrow(opt.association);
    std::string line;
    while(std::getline(assoc, line)) {
      if(line.empty() || line[0] == '#') {
        continue;
      }
      std::istringstream ss{line};
      double t = 0.0;
      std::string rgb;
      std::string depth;
      ss >> t >> rgb >> t >> depth;
      seq.timestamps.push_back(t);
      seq.left.push_back(opt.dataset + "/" + rgb);
      seq.right.push_back(opt.dataset + "/" + depth);
    }
    return seq;
  }
  std::ifstream list = OpenOrThrow(opt.dataset + "/rgb.txt");
  std::string line;
  while(std::getline(list, line)) {
    if(line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss{line};
    double t = 0.0;
    std::string rgb;
    ss >> t >> rgb;
    if(rgb.empty()) {
      continue;
    }
    seq.timestamps.push_back(t);
    seq.left.push_back(opt.dataset + "/" + rgb);
  }
  return seq;
}

Sequence LoadEuroc(const Options& opt) {
  if(opt.times.empty()) {
    throw std::runtime_error("--format euroc needs --times <EuRoC_TimeStamps file>");
  }
  Sequence seq;
  std::ifstream times = OpenOrThrow(opt.times);
  std::string line;
  while(std::getline(times, line)) {
    if(line.empty()) {
      continue;
    }
    seq.left.push_back(opt.dataset + "/cam0/data/" + line + ".png");
    if(opt.sensor == ORB_SLAM2::System::STEREO) {
      seq.right.push_back(opt.dataset + "/cam1/data/" + line + ".png");
    }
    std::istringstream ss{line};
    double t = 0.0;
    ss >> t;
    seq.timestamps.push_back(t / 1e9);
  }
  return seq;
}

Sequence LoadSequence(const Options& opt) {
  switch(opt.format) {
  case Format::kKitti:
    return LoadKitti(opt);
  case Format::kTum:
    return LoadTum(opt);
  case Format::kEuroc:
    return LoadEuroc(opt);
  }
  throw std::runtime_error("unreachable format");
}

// One frame of the dataset through the tracker. Returns Tcw (empty if lost).
cv::Mat TrackOne(ORB_SLAM2::System& slam, const Options& opt, const Sequence& seq, std::size_t i) {
  const cv::Mat left = cv::imread(seq.left[i], cv::IMREAD_UNCHANGED);
  if(left.empty()) {
    throw std::runtime_error("cannot read image " + seq.left[i]);
  }
  const double t = seq.timestamps[i];
  switch(opt.sensor) {
  case ORB_SLAM2::System::MONOCULAR:
    return slam.TrackMonocular(left, t);
  case ORB_SLAM2::System::STEREO: {
    const cv::Mat right = cv::imread(seq.right[i], cv::IMREAD_UNCHANGED);
    if(right.empty()) {
      throw std::runtime_error("cannot read image " + seq.right[i]);
    }
    return slam.TrackStereo(left, right, t);
  }
  case ORB_SLAM2::System::RGBD: {
    const cv::Mat depth = cv::imread(seq.right[i], cv::IMREAD_UNCHANGED);
    if(depth.empty()) {
      throw std::runtime_error("cannot read depth map " + seq.right[i]);
    }
    return slam.TrackRGBD(left, depth, t);
  }
  }
  return cv::Mat{};
}

void SaveTrajectory(ORB_SLAM2::System& slam, const Options& opt) {
  if(opt.trajectory.empty()) {
    return;
  }
  if(opt.sensor == ORB_SLAM2::System::MONOCULAR) {
    slam.SaveKeyFrameTrajectoryTUM(opt.trajectory);
  } else if(opt.format == Format::kKitti) {
    slam.SaveTrajectoryKITTI(opt.trajectory);
  } else {
    slam.SaveTrajectoryTUM(opt.trajectory);
  }
}

}    // namespace

int main(int argc, char** argv) {
  // '\n' rather than std::endl everywhere (SL.io.50), so unbuffer explicitly or a
  // redirected long run prints nothing until it exits.
  std::cout << std::unitbuf;

  try {
    const Options opt = ParseArgs(argc, argv);
    const Sequence seq = LoadSequence(opt);
    if(seq.left.empty()) {
      throw std::runtime_error("no images found under " + opt.dataset);
    }
    std::cout << "sequence: " << seq.left.size() << " frames\n";

    // Frame::mnId increments once per image, so the index is the frame id the
    // snapshot reports. This is what lets the GUI name and re-read keyframe images.
    orbslam_viewer::ImagePaths image_paths;
    image_paths.reserve(seq.left.size());
    for(std::size_t i = 0; i < seq.left.size(); ++i) {
      image_paths.emplace(static_cast<unsigned long>(i), seq.left[i]);
    }

    ORB_SLAM2::System slam(opt.vocabulary, opt.settings, opt.sensor, false);

    if(opt.headless) {
      for(std::size_t i = 0; i < seq.left.size(); ++i) {
        TrackOne(slam, opt, seq, i);
      }
      slam.Shutdown();
      SaveTrajectory(slam, opt);
      return 0;
    }

    orbslam_viewer::Viewer viewer{image_paths, "ORB-SLAM2"};

    // GLFW owns the main thread; the dataset loop gets its own. The jthread's stop
    // token is what closing the window uses to cut a long sequence short.
    std::jthread slam_thread{[&](std::stop_token stop) {
      // An escaping exception here would std::terminate the process, so the loop
      // reports and returns instead; the window stays up showing the partial map.
      try {
        for(std::size_t i = 0; i < seq.left.size() && !stop.stop_requested(); ++i) {
          const cv::Mat Tcw = TrackOne(slam, opt, seq, i);
          viewer.Publish(orbslam_viewer::BuildSnapshot(slam, Tcw));
        }
        std::cout << "sequence finished\n";
      } catch(const std::exception& e) {
        std::cerr << "slam thread stopped: " << e.what() << '\n';
      }
    }};

    while(!viewer.ShouldClose()) {
      viewer.RenderFrame();
    }
    slam_thread.request_stop();
    slam_thread.join();

    slam.Shutdown();
    SaveTrajectory(slam, opt);
    return 0;
  } catch(const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
}

