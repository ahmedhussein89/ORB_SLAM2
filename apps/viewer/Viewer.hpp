// GLFW window + OpenGL scene + Dear ImGui chrome. Owns the main thread; the SLAM
// loop runs elsewhere and hands us snapshots.
//
// The layout is fixed, not dockable: a 56px header, a body splitting a GL viewport
// from a 440px right column, and a 44px footer. The GL scene is scissored into the
// viewport cell; everything else is ImGui drawn on top.
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
struct ImFont;

namespace orbslam_viewer {

// Screen-space rectangle in ImGui (top-left origin) pixels.
struct Rect {
  float x{0.0F}, y{0.0F}, w{0.0F}, h{0.0F};
};

class Viewer {
public:
  // `slam` is what wires the buttons: the localization toggle, Reset and Save map all
  // just set flags that the tracking thread consumes on its next Track*() call.
  Viewer(ORB_SLAM2::System& slam,
         const std::string& title,
         const std::string& dataset_tag,
         const std::string& vocab_tag,
         const std::string& orb_caption);
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
  void ApplyTheme();

  void DrawUi(const Snapshot& snap);
  void DrawHeader(const Snapshot& snap, const Rect& band);
  void DrawViewportOverlay(const Snapshot& snap);
  void DrawFramePanel(const Snapshot& snap);
  void DrawStats(const Snapshot& snap);
  void DrawLayers();
  void DrawFooter(const Snapshot& snap, const Rect& band);

  void DrawScene(float dpi_scale, int fb_height);
  void UploadGeometry(const Snapshot& snap);
  void HandleCameraInput(const Snapshot& snap);
  void PickKeyFrame(const Snapshot& snap);
  void SelectKeyFrame(const Snapshot& snap, long id);
  void ResetView();

  Eigen::Matrix4f ViewMatrix() const;
  Eigen::Matrix4f ProjectionMatrix() const;

  GLFWwindow* window_{nullptr};
  ORB_SLAM2::System& slam_;
  const std::string dataset_tag_;
  const std::string vocab_tag_;
  const std::string orb_caption_;

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

  // Fonts. Null means the fallback bitmap font is in use.
  ImFont* font_body_{nullptr};
  ImFont* font_small_{nullptr};
  ImFont* font_heading_{nullptr};
  ImFont* font_stat_{nullptr};

  // The GL viewport cell, recomputed every frame from the window size.
  Rect viewport_{};
  bool viewport_hovered_{false};

  // Orbit camera
  float yaw_{-0.6F};
  float pitch_{0.5F};
  float distance_{10.0F};
  Eigen::Vector3f target_{Eigen::Vector3f::Zero()};
  bool follow_camera_{true};

  // Selection: -1 = none. Hovering a frustum sets the same field.
  long selected_kf_{-1};
  long hovered_kf_{-1};
  std::unordered_set<PointId> highlight_;

  // ponytail: the camera path is accumulated here rather than carried in the
  // snapshot, so a dropped snapshot drops a vertex. Read
  // Tracking::mlRelativeFramePoses instead if the path ever has to be exact.
  std::vector<Eigen::Vector3f> trajectory_;
  unsigned long last_traj_frame_{~0UL};

  int texture_width_{0};
  int texture_height_{0};

  bool show_points_{true};
  bool show_key_frames_{true};
  bool show_covis_{true};
  bool show_spanning_{false};
  bool show_loop_{true};
  bool show_local_{true};
  float frustum_size_{0.5F};    // metres for stereo/RGB-D; monocular maps need the slider
  float point_size_{2.0F};
};

}    // namespace orbslam_viewer
