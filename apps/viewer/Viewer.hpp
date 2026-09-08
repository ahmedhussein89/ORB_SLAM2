// GLFW window + OpenGL scene + Dear ImGui panels. Owns the main thread; the SLAM
// loop runs elsewhere and hands us snapshots.
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "Snapshot.hpp"

struct GLFWwindow;

namespace orbslam_viewer {

class Viewer {
public:
  Viewer(const ImagePaths& image_paths, const std::string& title);
  ~Viewer();

  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;
  Viewer(Viewer&&) = delete;
  Viewer& operator=(Viewer&&) = delete;

  // Thread-safe; called from the SLAM thread after every tracked frame.
  void Publish(std::shared_ptr<const Snapshot> snapshot);

  bool ShouldClose() const;
  void RenderFrame();

private:
  void DrawScene(int fb_width, int fb_height);
  void DrawPanels(const Snapshot& snap);
  void DrawImagePanel(const Snapshot& snap);
  void UploadGeometry(const Snapshot& snap);
  void HandleCameraInput(const Snapshot& snap);
  void PickKeyFrame(const Snapshot& snap, int fb_width, int fb_height);
  void SelectKeyFrame(const Snapshot& snap, long id);
  const cv::Mat& KeyFrameImage(unsigned long frame_id);

  Eigen::Matrix4f ViewMatrix() const;
  Eigen::Matrix4f ProjectionMatrix(int width, int height) const;

  GLFWwindow* window_{nullptr};
  const ImagePaths& image_paths_;

  mutable std::mutex snapshot_mutex_;
  std::shared_ptr<const Snapshot> pending_;
  std::shared_ptr<const Snapshot> current_;
  const void* uploaded_{nullptr};    // identity of the snapshot already in the VBOs

  // GL objects
  std::uint32_t program_{0};
  std::int32_t mvp_location_{-1};
  std::uint32_t point_vao_{0}, point_vbo_{0};
  std::uint32_t line_vao_{0}, line_vbo_{0};
  std::uint32_t texture_{0};
  int point_count_{0};
  int line_vertex_count_{0};

  // Orbit camera
  float yaw_{-0.6F};
  float pitch_{0.5F};
  float distance_{10.0F};
  Eigen::Vector3f target_{Eigen::Vector3f::Zero()};
  bool follow_camera_{true};

  // Selection: -1 = none. Hovering a frustum or a table row sets the same field.
  long selected_kf_{-1};
  long hovered_kf_{-1};
  std::unordered_set<PointId> highlight_;

  // ponytail: 4-entry LRU of decoded keyframe images (KeyFrame keeps no pixels, so
  // we re-read from disk). Grow it only if scrubbing the table drags.
  std::vector<std::pair<unsigned long, cv::Mat>> image_cache_;
  int texture_width_{0};
  int texture_height_{0};
  unsigned long texture_key_{~0UL};

  bool show_points_{true};
  bool show_key_frames_{true};
  bool show_graph_{true};
  float frustum_size_{0.5F};    // metres for stereo/RGB-D; monocular maps need the slider
  float point_size_{2.0F};
};

}    // namespace orbslam_viewer
