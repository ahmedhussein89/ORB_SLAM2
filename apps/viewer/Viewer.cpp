#include "Viewer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <utility>

#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>

#include <GLFW/glfw3.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "orbslam/tracking/Tracking.hpp"

namespace orbslam_viewer {
namespace {

constexpr int kInitialWidth = 1440;
constexpr int kInitialHeight = 900;
constexpr float kPickRadiusPx = 12.0F;
constexpr std::size_t kImageCacheSize = 4;
constexpr float kNearPlane = 0.05F;
constexpr float kFarPlane = 5000.0F;
constexpr float kFovYRadians = 0.8F;

const char* const kVertexShader = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMvp;
out vec3 vColor;
void main() {
  vColor = aColor;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

const char* const kFragmentShader = R"(#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main() { FragColor = vec4(vColor, 1.0); }
)";

std::uint32_t CompileShader(GLenum type, const char* source) {
  const std::uint32_t shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint ok = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if(ok == GL_FALSE) {
    char log[1024] = {};
    glGetShaderInfoLog(shader, sizeof(log) - 1, nullptr, log);
    throw std::runtime_error(std::string{"shader compilation failed: "} + log);
  }
  return shader;
}

// Apply a column-major 4x4 to a point.
std::array<float, 3> Transform(const std::array<float, 16>& m, float x, float y, float z) {
  return {m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13], m[2] * x + m[6] * y + m[10] * z + m[14]};
}

void PushVertex(std::vector<float>& out, const std::array<float, 3>& p, const std::array<float, 3>& c) {
  out.insert(out.end(), {p[0], p[1], p[2], c[0], c[1], c[2]});
}

const char* StateName(int state) {
  switch(state) {
  case ORB_SLAM2::Tracking::SYSTEM_NOT_READY:
    return "SYSTEM_NOT_READY";
  case ORB_SLAM2::Tracking::NO_IMAGES_YET:
    return "NO_IMAGES_YET";
  case ORB_SLAM2::Tracking::NOT_INITIALIZED:
    return "NOT_INITIALIZED";
  case ORB_SLAM2::Tracking::OK:
    return "OK";
  case ORB_SLAM2::Tracking::LOST:
    return "LOST";
  default:
    return "UNKNOWN";
  }
}

}    // namespace

Viewer::Viewer(const ImagePaths& image_paths, const std::string& title) : image_paths_(image_paths) {
  if(glfwInit() == GLFW_FALSE) {
    throw std::runtime_error("glfwInit failed (is DISPLAY/WAYLAND_DISPLAY set?)");
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  window_ = glfwCreateWindow(kInitialWidth, kInitialHeight, title.c_str(), nullptr, nullptr);
  if(window_ == nullptr) {
    glfwTerminate();
    throw std::runtime_error("glfwCreateWindow failed: no OpenGL 3.3 core context");
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window_, this);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  const std::uint32_t vertex = CompileShader(GL_VERTEX_SHADER, kVertexShader);
  const std::uint32_t fragment = CompileShader(GL_FRAGMENT_SHADER, kFragmentShader);
  program_ = glCreateProgram();
  glAttachShader(program_, vertex);
  glAttachShader(program_, fragment);
  glLinkProgram(program_);
  GLint linked = GL_FALSE;
  glGetProgramiv(program_, GL_LINK_STATUS, &linked);
  if(linked == GL_FALSE) {
    char log[1024] = {};
    glGetProgramInfoLog(program_, sizeof(log) - 1, nullptr, log);
    throw std::runtime_error(std::string{"shader link failed: "} + log);
  }
  glDeleteShader(vertex);
  glDeleteShader(fragment);
  mvp_location_ = glGetUniformLocation(program_, "uMvp");

  const auto setup = [](std::uint32_t& vao, std::uint32_t& vbo) {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
  };
  setup(point_vao_, point_vbo_);
  setup(line_vao_, line_vbo_);

  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);
}

Viewer::~Viewer() {
  glDeleteTextures(1, &texture_);
  glDeleteBuffers(1, &point_vbo_);
  glDeleteBuffers(1, &line_vbo_);
  glDeleteVertexArrays(1, &point_vao_);
  glDeleteVertexArrays(1, &line_vao_);
  glDeleteProgram(program_);
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window_);
  glfwTerminate();
}

void Viewer::Publish(std::shared_ptr<const Snapshot> snapshot) {
  const std::scoped_lock lock(snapshot_mutex_);
  pending_ = std::move(snapshot);
}

bool Viewer::ShouldClose() const {
  return glfwWindowShouldClose(window_) != 0;
}

Eigen::Matrix4f Viewer::ViewMatrix() const {
  const Eigen::Vector3f eye = target_ +
      distance_ * Eigen::Vector3f{std::cos(pitch_) * std::sin(yaw_), std::sin(pitch_), std::cos(pitch_) * std::cos(yaw_)};
  const Eigen::Vector3f forward = (target_ - eye).normalized();
  // ORB-SLAM2's world has +y down, so the natural "up" for a camera looking at the
  // map is -y; without this the whole trajectory renders upside down.
  const Eigen::Vector3f world_up{0.0F, -1.0F, 0.0F};
  Eigen::Vector3f right = forward.cross(world_up);
  if(right.norm() < 1e-6F) {
    right = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
  }
  right.normalize();
  const Eigen::Vector3f up = right.cross(forward);

  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view.block<1, 3>(0, 0) = right.transpose();
  view.block<1, 3>(1, 0) = up.transpose();
  view.block<1, 3>(2, 0) = -forward.transpose();
  view(0, 3) = -right.dot(eye);
  view(1, 3) = -up.dot(eye);
  view(2, 3) = forward.dot(eye);
  return view;
}

Eigen::Matrix4f Viewer::ProjectionMatrix(int width, int height) const {
  const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
  const float f = 1.0F / std::tan(kFovYRadians * 0.5F);
  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / aspect;
  proj(1, 1) = f;
  proj(2, 2) = (kFarPlane + kNearPlane) / (kNearPlane - kFarPlane);
  proj(2, 3) = (2.0F * kFarPlane * kNearPlane) / (kNearPlane - kFarPlane);
  proj(3, 2) = -1.0F;
  return proj;
}

void Viewer::SelectKeyFrame(const Snapshot& snap, long id) {
  if(selected_kf_ == id) {
    return;
  }
  selected_kf_ = id;
  highlight_.clear();
  if(id >= 0) {
    const auto it = std::find_if(
        snap.key_frames.begin(), snap.key_frames.end(), [id](const KeyFrameView& v) { return static_cast<long>(v.id) == id; });
    if(it != snap.key_frames.end()) {
      highlight_.insert(it->point_ids.begin(), it->point_ids.end());
    }
  }
  uploaded_ = nullptr;    // colours changed: force a re-upload
}

void Viewer::UploadGeometry(const Snapshot& snap) {
  constexpr std::array<float, 3> kGrey{0.35F, 0.35F, 0.35F};
  constexpr std::array<float, 3> kRed{0.9F, 0.15F, 0.15F};
  constexpr std::array<float, 3> kYellow{1.0F, 0.85F, 0.1F};
  constexpr std::array<float, 3> kBlue{0.35F, 0.55F, 0.95F};
  constexpr std::array<float, 3> kGreen{0.1F, 0.9F, 0.2F};
  constexpr std::array<float, 3> kGraph{0.18F, 0.35F, 0.28F};

  std::vector<float> points;
  points.reserve(snap.point_ids.size() * 6);
  for(std::size_t i = 0; i < snap.point_ids.size(); ++i) {
    const std::array<float, 3> p{snap.point_xyz[3 * i], snap.point_xyz[3 * i + 1], snap.point_xyz[3 * i + 2]};
    const bool lit = !highlight_.empty() && highlight_.count(snap.point_ids[i]) != 0U;
    PushVertex(points, p, lit ? kYellow : (snap.point_ref[i] != 0U ? kRed : kGrey));
  }
  point_count_ = static_cast<int>(snap.point_ids.size());
  glBindBuffer(GL_ARRAY_BUFFER, point_vbo_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(points.size() * sizeof(float)),
               points.empty() ? nullptr : points.data(),
               GL_DYNAMIC_DRAW);

  std::vector<float> lines;
  // Frustum corners in camera coordinates, at frustum_size_ metres depth.
  const float d = frustum_size_;
  const float w = snap.fx > 0.0F ? d * (snap.width * 0.5F) / snap.fx : d * 0.7F;
  const float h = snap.fy > 0.0F ? d * (snap.height * 0.5F) / snap.fy : d * 0.5F;
  const std::array<std::array<float, 3>, 4> corners{{{w, h, d}, {w, -h, d}, {-w, -h, d}, {-w, h, d}}};

  const auto push_frustum = [&](const std::array<float, 16>& Twc, const std::array<float, 3>& colour) {
    const std::array<float, 3> origin = Transform(Twc, 0.0F, 0.0F, 0.0F);
    std::array<std::array<float, 3>, 4> world{};
    for(std::size_t i = 0; i < 4; ++i) {
      world[i] = Transform(Twc, corners[i][0], corners[i][1], corners[i][2]);
    }
    for(std::size_t i = 0; i < 4; ++i) {
      PushVertex(lines, origin, colour);
      PushVertex(lines, world[i], colour);
      PushVertex(lines, world[i], colour);
      PushVertex(lines, world[(i + 1) % 4], colour);
    }
  };

  if(show_key_frames_) {
    for(const KeyFrameView& kf : snap.key_frames) {
      const bool lit = static_cast<long>(kf.id) == selected_kf_;
      push_frustum(kf.Twc, lit ? kYellow : kBlue);
    }
  }
  if(show_graph_) {
    for(const std::array<float, 6>& e : snap.graph_edges) {
      PushVertex(lines, {e[0], e[1], e[2]}, kGraph);
      PushVertex(lines, {e[3], e[4], e[5]}, kGraph);
    }
  }
  if(snap.pose_valid) {
    push_frustum(snap.Twc, kGreen);
  }

  line_vertex_count_ = static_cast<int>(lines.size() / 6);
  glBindBuffer(GL_ARRAY_BUFFER, line_vbo_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(lines.size() * sizeof(float)),
               lines.empty() ? nullptr : lines.data(),
               GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  uploaded_ = static_cast<const void*>(&snap);
}

void Viewer::HandleCameraInput(const Snapshot& snap) {
  const ImGuiIO& io = ImGui::GetIO();
  if(follow_camera_ && snap.pose_valid) {
    target_ = Eigen::Vector3f{snap.Twc[12], snap.Twc[13], snap.Twc[14]};
  }
  if(io.WantCaptureMouse) {
    return;
  }
  if(ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    const ImVec2 delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
    ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);
    yaw_ -= delta.x * 0.005F;
    pitch_ = std::clamp(pitch_ + delta.y * 0.005F, -1.5F, 1.5F);
  }
  if(ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    const ImVec2 delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
    ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
    const Eigen::Matrix4f view = ViewMatrix();
    const Eigen::Vector3f right = view.block<1, 3>(0, 0).transpose();
    const Eigen::Vector3f up = view.block<1, 3>(1, 0).transpose();
    const float scale = distance_ * 0.002F;
    target_ += (-delta.x * scale) * right + (delta.y * scale) * up;
    follow_camera_ = false;
  }
  if(io.MouseWheel != 0.0F) {
    distance_ = std::clamp(distance_ * std::pow(0.9F, io.MouseWheel), 0.05F, 5000.0F);
  }
}

void Viewer::PickKeyFrame(const Snapshot& snap, int width, int height) {
  hovered_kf_ = -1;
  if(ImGui::GetIO().WantCaptureMouse || snap.key_frames.empty()) {
    return;
  }
  const ImVec2 mouse = ImGui::GetMousePos();
  const Eigen::Matrix4f mvp = ProjectionMatrix(width, height) * ViewMatrix();
  float best = kPickRadiusPx;
  // ponytail: O(n) scan over keyframe centres per frame. Thousands of keyframes is
  // microseconds; add a spatial grid only if a 10k-keyframe map ever drags.
  for(const KeyFrameView& kf : snap.key_frames) {
    const Eigen::Vector4f clip = mvp * Eigen::Vector4f{kf.center[0], kf.center[1], kf.center[2], 1.0F};
    if(clip.w() <= 0.0F) {
      continue;
    }
    const float sx = (clip.x() / clip.w() * 0.5F + 0.5F) * static_cast<float>(width);
    const float sy = (1.0F - (clip.y() / clip.w() * 0.5F + 0.5F)) * static_cast<float>(height);
    const float dist = std::hypot(sx - mouse.x, sy - mouse.y);
    if(dist < best) {
      best = dist;
      hovered_kf_ = static_cast<long>(kf.id);
    }
  }
  if(hovered_kf_ >= 0) {
    ImGui::SetTooltip("KeyFrame %ld", hovered_kf_);
    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      SelectKeyFrame(snap, hovered_kf_);
    }
  }
}

void Viewer::DrawScene(int fb_width, int fb_height) {
  glViewport(0, 0, fb_width, fb_height);
  glClearColor(0.08F, 0.09F, 0.11F, 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  const Eigen::Matrix4f mvp = ProjectionMatrix(fb_width, fb_height) * ViewMatrix();
  glUseProgram(program_);
  glUniformMatrix4fv(mvp_location_, 1, GL_FALSE, mvp.data());

  if(show_points_ && point_count_ > 0) {
    glPointSize(point_size_);
    glBindVertexArray(point_vao_);
    glDrawArrays(GL_POINTS, 0, point_count_);
  }
  if(line_vertex_count_ > 0) {
    glBindVertexArray(line_vao_);
    glDrawArrays(GL_LINES, 0, line_vertex_count_);
  }
  glBindVertexArray(0);
}

const cv::Mat& Viewer::KeyFrameImage(unsigned long frame_id) {
  static const cv::Mat kEmpty;
  const auto hit = std::find_if(
      image_cache_.begin(), image_cache_.end(), [frame_id](const auto& e) { return e.first == frame_id; });
  if(hit != image_cache_.end()) {
    return hit->second;
  }
  const auto path = image_paths_.find(frame_id);
  if(path == image_paths_.end()) {
    return kEmpty;
  }
  cv::Mat image = cv::imread(path->second, cv::IMREAD_GRAYSCALE);
  if(image.empty()) {
    return kEmpty;
  }
  if(image_cache_.size() >= kImageCacheSize) {
    image_cache_.erase(image_cache_.begin());
  }
  image_cache_.emplace_back(frame_id, std::move(image));
  return image_cache_.back().second;
}

void Viewer::DrawImagePanel(const Snapshot& snap) {
  const KeyFrameView* selected = nullptr;
  if(selected_kf_ >= 0) {
    const auto it = std::find_if(snap.key_frames.begin(), snap.key_frames.end(), [this](const KeyFrameView& v) {
      return static_cast<long>(v.id) == selected_kf_;
    });
    if(it != snap.key_frames.end()) {
      selected = &(*it);
    }
  }

  const cv::Mat& image = selected != nullptr ? KeyFrameImage(selected->frame_id) : snap.current_image;
  const unsigned long key = selected != nullptr ? selected->frame_id : ~0UL;
  const std::string label = selected != nullptr ?
      (image_paths_.count(selected->frame_id) != 0U ?
           std::filesystem::path{image_paths_.at(selected->frame_id)}.filename().string() :
           std::string{"keyframe "} + std::to_string(selected->id)) :
      std::string{"live frame "} + std::to_string(snap.frame_id);

  ImGui::Begin("Image");
  ImGui::TextUnformatted(label.c_str());
  if(selected != nullptr && ImGui::Button("Back to live frame")) {
    SelectKeyFrame(snap, -1);
  }
  if(image.empty()) {
    ImGui::TextUnformatted("(no image)");
    ImGui::End();
    return;
  }

  // The live frame changes every tick, so it is re-uploaded unconditionally; a
  // selected keyframe is uploaded once.
  if(key != texture_key_ || key == ~0UL) {
    cv::Mat rgb;
    cv::cvtColor(image, rgb, image.channels() == 1 ? cv::COLOR_GRAY2RGB : cv::COLOR_BGR2RGB);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgb.cols, rgb.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data);
    texture_width_ = rgb.cols;
    texture_height_ = rgb.rows;
    texture_key_ = key;
  }

  const float avail = std::max(ImGui::GetContentRegionAvail().x, 64.0F);
  const float scale = avail / static_cast<float>(texture_width_);
  const ImVec2 size{avail, static_cast<float>(texture_height_) * scale};
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  ImGui::Image(static_cast<ImTextureID>(static_cast<std::intptr_t>(texture_)), size);

  ImDrawList* const draw = ImGui::GetWindowDrawList();
  constexpr ImU32 kGreen = IM_COL32(40, 220, 60, 255);
  constexpr ImU32 kBlue = IM_COL32(90, 140, 240, 255);
  constexpr ImU32 kYellow = IM_COL32(255, 216, 25, 255);
  const float r = 3.0F;
  if(selected != nullptr) {
    // Keypoint coordinates come straight from KeyFrame::mvKeysUn -- no reprojection.
    std::vector<unsigned char> mapped(selected->keypoints.size(), 0);
    for(const int idx : selected->keypoint_index) {
      if(idx >= 0 && static_cast<std::size_t>(idx) < mapped.size()) {
        mapped[static_cast<std::size_t>(idx)] = 1;
      }
    }
    for(std::size_t i = 0; i < selected->keypoints.size(); ++i) {
      const ImVec2 p{origin.x + selected->keypoints[i][0] * scale, origin.y + selected->keypoints[i][1] * scale};
      draw->AddCircle(p, r, mapped[i] != 0U ? kYellow : kBlue);
    }
  } else {
    for(std::size_t i = 0; i < snap.current_keys.size(); ++i) {
      const ImVec2 p{origin.x + snap.current_keys[i][0] * scale, origin.y + snap.current_keys[i][1] * scale};
      draw->AddCircle(p, r, snap.current_mapped[i] != 0U ? kGreen : kBlue);
    }
  }
  ImGui::End();
}

void Viewer::DrawPanels(const Snapshot& snap) {
  ImGui::Begin("SLAM");
  ImGui::Text("state       %s", StateName(snap.state));
  ImGui::Text("frame       %lu", snap.frame_id);
  ImGui::Text("keyframes   %zu", snap.n_key_frames);
  ImGui::Text("map points  %zu", snap.n_map_points);
  ImGui::Text("render      %.1f FPS", static_cast<double>(ImGui::GetIO().Framerate));
  ImGui::Separator();
  ImGui::Checkbox("points", &show_points_);
  ImGui::SameLine();
  ImGui::Checkbox("keyframes", &show_key_frames_);
  ImGui::SameLine();
  ImGui::Checkbox("graph", &show_graph_);
  if(ImGui::Checkbox("follow camera", &follow_camera_) || ImGui::SliderFloat("frustum", &frustum_size_, 0.01F, 5.0F, "%.2f") ||
     ImGui::SliderFloat("point size", &point_size_, 1.0F, 6.0F, "%.1f")) {
    uploaded_ = nullptr;
  }
  ImGui::TextUnformatted("LMB orbit  |  RMB pan  |  wheel zoom  |  hover a camera to highlight");
  ImGui::End();

  ImGui::Begin("Keyframes");
  constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
      ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
  if(ImGui::BeginTable("kf_table", 6, kFlags)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("image");
    ImGui::TableSetupColumn("kf");
    ImGui::TableSetupColumn("frame");
    ImGui::TableSetupColumn("time");
    ImGui::TableSetupColumn("features");
    ImGui::TableSetupColumn("tracked");
    ImGui::TableHeadersRow();

    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(snap.key_frames.size()));
    while(clipper.Step()) {
      for(int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row) {
        const KeyFrameView& kf = snap.key_frames[static_cast<std::size_t>(row)];
        const auto path = image_paths_.find(kf.frame_id);
        const std::string name = path != image_paths_.end() ? std::filesystem::path{path->second}.filename().string() :
                                                              std::string{"-"};
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        const bool selected = static_cast<long>(kf.id) == selected_kf_;
        if(ImGui::Selectable(name.c_str(), selected, ImGuiSelectableFlags_SpanAllColumns)) {
          SelectKeyFrame(snap, selected ? -1 : static_cast<long>(kf.id));
        }
        if(ImGui::IsItemHovered() && !selected) {
          hovered_kf_ = static_cast<long>(kf.id);
        }
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%lu", kf.id);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%lu", kf.frame_id);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.3f", kf.timestamp);
        ImGui::TableSetColumnIndex(4);
        ImGui::Text("%d", kf.n_features);
        ImGui::TableSetColumnIndex(5);
        ImGui::Text("%d", kf.n_tracked);
      }
    }
    ImGui::EndTable();
  }
  ImGui::End();

  DrawImagePanel(snap);
}

void Viewer::RenderFrame() {
  {
    const std::scoped_lock lock(snapshot_mutex_);
    if(pending_ != nullptr) {
      current_ = std::move(pending_);
      pending_.reset();
      uploaded_ = nullptr;
    }
  }
  static const Snapshot kEmpty;
  const Snapshot& snap = current_ != nullptr ? *current_ : kEmpty;

  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window_, &width, &height);

  HandleCameraInput(snap);
  DrawPanels(snap);
  PickKeyFrame(snap, width, height);
  // Hovering (a frustum or a table row) drives the same highlight as clicking.
  if(hovered_kf_ >= 0 && selected_kf_ < 0) {
    highlight_.clear();
    const auto it = std::find_if(snap.key_frames.begin(), snap.key_frames.end(), [this](const KeyFrameView& v) {
      return static_cast<long>(v.id) == hovered_kf_;
    });
    if(it != snap.key_frames.end()) {
      highlight_.insert(it->point_ids.begin(), it->point_ids.end());
      uploaded_ = nullptr;
    }
  } else if(hovered_kf_ < 0 && selected_kf_ < 0 && !highlight_.empty()) {
    highlight_.clear();
    uploaded_ = nullptr;
  }

  if(uploaded_ != static_cast<const void*>(&snap)) {
    UploadGeometry(snap);
  }
  DrawScene(width, height);

  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_);
}

}    // namespace orbslam_viewer
