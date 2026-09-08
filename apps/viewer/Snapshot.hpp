// Visualization snapshot: the only thing that crosses from the SLAM threads to
// the render thread. No MapPoint*/KeyFrame* ever escapes -- the map owns those and
// may SetBadFlag()/Replace() them at any moment.
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>

#include "orbslam/system/System.hpp"

namespace orbslam_viewer {

using PointId = unsigned long;

struct KeyFrameView {
  PointId id{};
  std::array<float, 16> Twc{};    // column-major, ready for glUniformMatrix4fv
  std::array<float, 3> center{};
  int n_tracked{};
  // MapPoint ids this keyframe observes. Highlighting a picked frustum is all
  // these are for; the keypoints themselves stay in the map.
  std::vector<PointId> point_ids;
};

struct Snapshot {
  int state{-1};    // ORB_SLAM2::Tracking::eTrackingState
  bool pose_valid{false};
  std::array<float, 16> Twc{};

  std::vector<float> point_xyz;            // flat x,y,z, straight into a VBO
  std::vector<PointId> point_ids;          // parallel to point_xyz / 3
  std::vector<unsigned char> point_ref;    // 1 = local reference point (drawn light)

  std::vector<KeyFrameView> key_frames;
  // The three graph layers, each toggled on its own. Element is x1,y1,z1,x2,y2,z2.
  std::vector<std::array<float, 6>> covis_edges;
  std::vector<std::array<float, 6>> spanning_edges;
  std::vector<std::array<float, 6>> loop_edges;

  cv::Mat current_image;    // deep copy: Tracking::mImGray aliases the caller's buffer
  std::vector<std::array<float, 2>> current_keys;
  std::vector<unsigned char> current_mapped;    // 1 = matched map point, 0 = VO only
  unsigned long frame_id{};
  double timestamp{};
  double track_ms{};    // measured around Track*() by the caller
  int sensor{};         // ORB_SLAM2::System::eSensor
  bool localization_mode{false};

  std::size_t n_key_frames{};
  std::size_t n_map_points{};
  int matches_inliers{};
  std::size_t n_local_key_frames{};
  std::size_t n_local_map_points{};
  unsigned long last_reloc_frame{};
  std::size_t n_loop_closures{};    // keyframes carrying at least one loop edge
  unsigned long last_loop_kf{};

  // Intrinsics of the last frame, for drawing frustums at the right aspect.
  float fx{1.0F}, fy{1.0F}, cx{0.0F}, cy{0.0F}, width{640.0F}, height{480.0F};
};

// Runs on the SLAM thread, right after Track*() returns. `Tcw` is that return value,
// `track_ms` the wall time that call took.
std::shared_ptr<const Snapshot> BuildSnapshot(const ORB_SLAM2::System& slam, const cv::Mat& Tcw, double track_ms);

}    // namespace orbslam_viewer
