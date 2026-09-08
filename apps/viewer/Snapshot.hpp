// Visualization snapshot: the only thing that crosses from the SLAM threads to
// the render thread. No MapPoint*/KeyFrame* ever escapes -- the map owns those and
// may SetBadFlag()/Replace() them at any moment.
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core/core.hpp>

#include "orbslam/system/System.hpp"

namespace orbslam_viewer {

using PointId = unsigned long;

struct KeyFrameView {
  PointId id{};
  unsigned long frame_id{};
  double timestamp{};
  std::array<float, 16> Twc{};    // column-major, ready for glUniformMatrix4fv
  std::array<float, 3> center{};
  int n_features{};
  int n_tracked{};
  // MapPoint ids this keyframe observes, and the keypoint index each was seen at.
  // Parallel arrays; index i of point_ids matches index i of keypoint_index.
  std::vector<PointId> point_ids;
  std::vector<int> keypoint_index;
  std::vector<std::array<float, 2>> keypoints;    // mvKeysUn, for the image overlay
};

struct Snapshot {
  int state{-1};    // ORB_SLAM2::Tracking::eTrackingState
  bool pose_valid{false};
  std::array<float, 16> Twc{};

  std::vector<float> point_xyz;            // flat x,y,z, straight into a VBO
  std::vector<PointId> point_ids;          // parallel to point_xyz / 3
  std::vector<unsigned char> point_ref;    // 1 = local reference point (drawn red)

  std::vector<KeyFrameView> key_frames;
  std::vector<std::array<float, 6>> graph_edges;    // covisibility + spanning tree + loop

  cv::Mat current_image;    // deep copy: Tracking::mImGray aliases the caller's buffer
  std::vector<std::array<float, 2>> current_keys;
  std::vector<unsigned char> current_mapped;    // 1 = matched map point, 0 = VO only
  unsigned long frame_id{};

  std::size_t n_key_frames{};
  std::size_t n_map_points{};

  // Intrinsics of the last frame, for drawing frustums at the right aspect.
  float fx{1.0F}, fy{1.0F}, cx{0.0F}, cy{0.0F}, width{640.0F}, height{480.0F};
};

// Runs on the SLAM thread, right after Track*() returns. `Tcw` is that return value.
std::shared_ptr<const Snapshot> BuildSnapshot(const ORB_SLAM2::System& slam, const cv::Mat& Tcw);

// frame id -> image path, filled by the dataset loader so the GUI can name and
// re-read keyframe images (KeyFrame itself stores no pixels).
using ImagePaths = std::unordered_map<unsigned long, std::string>;

}    // namespace orbslam_viewer
