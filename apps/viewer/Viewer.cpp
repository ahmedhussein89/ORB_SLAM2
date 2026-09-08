#include "Viewer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>

#include <GLFW/glfw3.h>
#include <opencv2/imgproc.hpp>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "orbslam/tracking/Tracking.hpp"

namespace orbslam_viewer {
namespace {

constexpr int kInitialWidth = 1600;
constexpr int kInitialHeight = 1000;
constexpr float kPickRadiusPx = 12.0F;
constexpr float kNearPlane = 0.05F;
constexpr float kFarPlane = 5000.0F;
constexpr float kFovYRadians = 0.8F;

// Layout bands, in the mockup's units.
constexpr float kHeaderH = 56.0F;
constexpr float kFooterH = 44.0F;
constexpr float kPadX = 28.0F;
constexpr float kPadY = 24.0F;
constexpr float kRightW = 440.0F;
constexpr float kGap = 24.0F;
constexpr float kFrameH = 280.0F;

// Palette -- the mockup's design tokens with the roles swapped for a dark ground:
// the ink becomes the background, and the accent steps up the ramp (400 instead of
// 500) so it still carries against it. 0xAABBGGRR for ImU32.
constexpr ImU32 kBg = IM_COL32(0x1A, 0x1C, 0x1D, 0xFF);
constexpr ImU32 kSurface = IM_COL32(0x26, 0x28, 0x2A, 0xFF);
constexpr ImU32 kText = IM_COL32(0xF2, 0xF2, 0xF3, 0xFF);
constexpr ImU32 kMuted = IM_COL32(0x98, 0x98, 0x9B, 0xFF);
constexpr ImU32 kAccent = IM_COL32(0x94, 0xBC, 0xE3, 0xFF);
constexpr ImU32 kAccent700 = IM_COL32(0xB5, 0xD9, 0xFD, 0xFF);
constexpr ImU32 kAccent200 = IM_COL32(0xD6, 0xEB, 0xFF, 0xFF);
constexpr ImU32 kAccent100 = IM_COL32(0x1D, 0x2D, 0x3D, 0xFF);
constexpr ImU32 kDivider = IM_COL32(0xF2, 0xF2, 0xF3, 0x2E);
constexpr ImU32 kHudText = IM_COL32(0xD6, 0xEB, 0xFF, 0xFF);
constexpr ImU32 kHudLine = IM_COL32(0x41, 0x61, 0x80, 0xFF);

// GL colours, RGB floats. The viewport is accent-900.
constexpr std::array<float, 3> kClearNavy{0.114F, 0.176F, 0.239F};
constexpr std::array<float, 3> kPoint{0.580F, 0.737F, 0.890F};         // accent-400
constexpr std::array<float, 3> kLocalPoint{0.949F, 0.949F, 0.953F};    // bg, drawn bright
constexpr std::array<float, 3> kHighlight{1.0F, 0.85F, 0.1F};
constexpr std::array<float, 3> kFrustum{0.580F, 0.737F, 0.890F};
constexpr std::array<float, 3> kCurrentCam{0.949F, 0.949F, 0.953F};
constexpr std::array<float, 3> kCovis{0.255F, 0.380F, 0.502F};       // accent-700
constexpr std::array<float, 3> kSpanning{0.349F, 0.502F, 0.651F};    // accent
constexpr std::array<float, 3> kLoop{0.839F, 0.922F, 1.0F};          // accent-200
constexpr std::array<float, 3> kTrajectory{0.839F, 0.922F, 1.0F};

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

const char* StateLabel(int state) {
  switch(state) {
  case ORB_SLAM2::Tracking::SYSTEM_NOT_READY:
    return "SYSTEM NOT READY";
  case ORB_SLAM2::Tracking::NO_IMAGES_YET:
    return "NO IMAGES YET";
  case ORB_SLAM2::Tracking::NOT_INITIALIZED:
    return "NOT INITIALIZED";
  case ORB_SLAM2::Tracking::OK:
    return "TRACKING OK";
  case ORB_SLAM2::Tracking::LOST:
    return "TRACKING LOST";
  default:
    return "UNKNOWN";
  }
}

ImU32 StateColor(int state) {
  switch(state) {
  case ORB_SLAM2::Tracking::OK:
    return kAccent;
  case ORB_SLAM2::Tracking::LOST:
    return kText;
  default:
    return kMuted;
  }
}

// The .blueprint registration marks: an 11px L at each corner, sitting outside the box.
void BlueprintCorners(ImDrawList* draw, const ImVec2& min, const ImVec2& max, ImU32 col) {
  constexpr float kLen = 11.0F;
  constexpr float kOff = 6.0F;
  const float xs[2] = {min.x - kOff, max.x + kOff};
  const float ys[2] = {min.y - kOff, max.y + kOff};
  for(const float x : xs) {
    for(const float y : ys) {
      draw->AddLine({x, y - kLen * 0.5F}, {x, y + kLen * 0.5F}, col);
      draw->AddLine({x - kLen * 0.5F, y}, {x + kLen * 0.5F, y}, col);
    }
  }
}

// A bordered .tag chip. Returns its width so chips can be laid out in a row.
float Chip(ImDrawList* draw, const ImVec2& pos, const char* text, ImU32 border, ImU32 fill, ImU32 text_col) {
  const ImVec2 size = ImGui::CalcTextSize(text);
  const ImVec2 max{pos.x + size.x + 20.0F, pos.y + size.y + 6.0F};
  if((fill & IM_COL32_A_MASK) != 0U) {
    draw->AddRectFilled(pos, max, fill);
  }
  if((border & IM_COL32_A_MASK) != 0U) {
    draw->AddRect(pos, max, border);
  }
  draw->AddText({pos.x + 10.0F, pos.y + 3.0F}, text_col, text);
  return size.x + 20.0F;
}

void LegendSwatch(ImDrawList* draw, const ImVec2& pos, ImU32 col, bool line, float thickness) {
  if(line) {
    draw->AddLine({pos.x, pos.y + 6.0F}, {pos.x + 14.0F, pos.y + 6.0F}, col, thickness);
  } else {
    draw->AddRectFilled({pos.x + 4.0F, pos.y + 3.0F}, {pos.x + 10.0F, pos.y + 9.0F}, col);
  }
}

}    // namespace

Viewer::Viewer(ORB_SLAM2::System& slam,
               const std::string& title,
               const std::string& dataset_tag,
               const std::string& vocab_tag,
               const std::string& orb_caption)
    : slam_(slam), dataset_tag_(dataset_tag), vocab_tag_(vocab_tag), orb_caption_(orb_caption) {
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
  ApplyTheme();
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

void Viewer::ApplyTheme() {
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = nullptr;    // the layout is fixed; nothing to remember

  // Barlow / Barlow Condensed are not installed anywhere; DejaVu is, on every
  // distro that ships fontconfig, and DejaVuSansCondensed is the closest match to
  // the mockup's condensed headings. Missing files fall back to the bitmap font.
  const char* const kBody = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";
  const char* const kCond = "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf";
  font_body_ = io.Fonts->AddFontFromFileTTF(kBody, 15.0F);
  font_small_ = io.Fonts->AddFontFromFileTTF(kBody, 11.0F);
  font_heading_ = io.Fonts->AddFontFromFileTTF(kCond, 19.0F);
  font_stat_ = io.Fonts->AddFontFromFileTTF(kCond, 26.0F);
  if(font_body_ == nullptr || font_small_ == nullptr || font_heading_ == nullptr || font_stat_ == nullptr) {
    io.Fonts->Clear();
    io.Fonts->AddFontDefault();
    font_body_ = font_small_ = font_heading_ = font_stat_ = nullptr;
  }

  ImGuiStyle& style = ImGui::GetStyle();
  ImGui::StyleColorsDark();
  style.WindowRounding = 0.0F;
  style.ChildRounding = 0.0F;
  style.FrameRounding = 0.0F;
  style.GrabRounding = 0.0F;
  style.PopupRounding = 0.0F;
  style.ScrollbarRounding = 0.0F;
  style.WindowBorderSize = 0.0F;
  style.ChildBorderSize = 1.0F;
  style.FrameBorderSize = 1.0F;
  style.ItemSpacing = ImVec2(8.0F, 8.0F);
  style.FramePadding = ImVec2(10.0F, 6.0F);
  style.CellPadding = ImVec2(12.0F, 10.0F);

  const auto set = [&style](ImGuiCol idx, ImU32 col) { style.Colors[idx] = ImGui::ColorConvertU32ToFloat4(col); };
  set(ImGuiCol_WindowBg, kBg);
  set(ImGuiCol_ChildBg, IM_COL32(0, 0, 0, 0));
  set(ImGuiCol_Text, kText);
  set(ImGuiCol_TextDisabled, kMuted);
  set(ImGuiCol_Border, kDivider);
  set(ImGuiCol_FrameBg, kSurface);
  set(ImGuiCol_FrameBgHovered, kAccent100);
  set(ImGuiCol_FrameBgActive, kAccent100);
  set(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
  set(ImGuiCol_ButtonHovered, IM_COL32(0xF2, 0xF2, 0xF3, 0x16));
  set(ImGuiCol_ButtonActive, IM_COL32(0xF2, 0xF2, 0xF3, 0x2C));
  set(ImGuiCol_CheckMark, kAccent);
  set(ImGuiCol_SliderGrab, kAccent);
  set(ImGuiCol_SliderGrabActive, kAccent700);
  set(ImGuiCol_TableBorderStrong, kDivider);
  set(ImGuiCol_TableBorderLight, kDivider);
  set(ImGuiCol_Separator, kDivider);
}

void Viewer::Publish(std::shared_ptr<const Snapshot> snapshot) {
  const std::scoped_lock lock(snapshot_mutex_);
  pending_ = std::move(snapshot);
}

bool Viewer::ShouldClose() const {
  return glfwWindowShouldClose(window_) != 0;
}

void Viewer::ResetView() {
  yaw_ = -0.6F;
  pitch_ = 0.5F;
  distance_ = 10.0F;
  target_ = Eigen::Vector3f::Zero();
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

Eigen::Matrix4f Viewer::ProjectionMatrix() const {
  const float aspect = viewport_.h > 0.0F ? viewport_.w / viewport_.h : 1.0F;
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
  std::vector<float> points;
  points.reserve(snap.point_ids.size() * 6);
  for(std::size_t i = 0; i < snap.point_ids.size(); ++i) {
    const std::array<float, 3> p{snap.point_xyz[3 * i], snap.point_xyz[3 * i + 1], snap.point_xyz[3 * i + 2]};
    const bool lit = !highlight_.empty() && highlight_.count(snap.point_ids[i]) != 0U;
    const bool local = show_local_ && snap.point_ref[i] != 0U;
    PushVertex(points, p, lit ? kHighlight : (local ? kLocalPoint : kPoint));
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

  const auto push_edges = [&lines](const std::vector<std::array<float, 6>>& edges, const std::array<float, 3>& colour) {
    for(const std::array<float, 6>& e : edges) {
      PushVertex(lines, {e[0], e[1], e[2]}, colour);
      PushVertex(lines, {e[3], e[4], e[5]}, colour);
    }
  };

  if(show_key_frames_) {
    for(const KeyFrameView& kf : snap.key_frames) {
      const bool lit = static_cast<long>(kf.id) == selected_kf_;
      push_frustum(kf.Twc, lit ? kHighlight : kFrustum);
    }
  }
  if(show_covis_) {
    push_edges(snap.covis_edges, kCovis);
  }
  if(show_spanning_) {
    push_edges(snap.spanning_edges, kSpanning);
  }
  if(show_loop_) {
    push_edges(snap.loop_edges, kLoop);
  }
  for(std::size_t i = 1; i < trajectory_.size(); ++i) {
    PushVertex(lines, {trajectory_[i - 1].x(), trajectory_[i - 1].y(), trajectory_[i - 1].z()}, kTrajectory);
    PushVertex(lines, {trajectory_[i].x(), trajectory_[i].y(), trajectory_[i].z()}, kTrajectory);
  }
  if(snap.pose_valid) {
    push_frustum(snap.Twc, kCurrentCam);
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
  if(!viewport_hovered_) {
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

void Viewer::PickKeyFrame(const Snapshot& snap) {
  hovered_kf_ = -1;
  if(!viewport_hovered_ || snap.key_frames.empty()) {
    return;
  }
  const ImVec2 mouse = ImGui::GetMousePos();
  const Eigen::Matrix4f mvp = ProjectionMatrix() * ViewMatrix();
  float best = kPickRadiusPx;
  // ponytail: O(n) scan over keyframe centres per frame. Thousands of keyframes is
  // microseconds; add a spatial grid only if a 10k-keyframe map ever drags.
  for(const KeyFrameView& kf : snap.key_frames) {
    const Eigen::Vector4f clip = mvp * Eigen::Vector4f{kf.center[0], kf.center[1], kf.center[2], 1.0F};
    if(clip.w() <= 0.0F) {
      continue;
    }
    const float sx = viewport_.x + (clip.x() / clip.w() * 0.5F + 0.5F) * viewport_.w;
    const float sy = viewport_.y + (1.0F - (clip.y() / clip.w() * 0.5F + 0.5F)) * viewport_.h;
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

void Viewer::DrawScene(float dpi_scale, int fb_height) {
  if(viewport_.w < 1.0F || viewport_.h < 1.0F) {
    return;
  }
  const GLint x = static_cast<GLint>(viewport_.x * dpi_scale);
  const GLint y = static_cast<GLint>(static_cast<float>(fb_height) - (viewport_.y + viewport_.h) * dpi_scale);
  const GLsizei w = static_cast<GLsizei>(viewport_.w * dpi_scale);
  const GLsizei h = static_cast<GLsizei>(viewport_.h * dpi_scale);

  glViewport(x, y, w, h);
  glEnable(GL_SCISSOR_TEST);
  glScissor(x, y, w, h);
  glClearColor(kClearNavy[0], kClearNavy[1], kClearNavy[2], 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  const Eigen::Matrix4f mvp = ProjectionMatrix() * ViewMatrix();
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
  glDisable(GL_SCISSOR_TEST);
}

void Viewer::DrawHeader(const Snapshot& snap, const Rect& band) {
  ImDrawList* const draw = ImGui::GetWindowDrawList();
  draw->AddLine({band.x, band.y + band.h}, {band.x + band.w, band.y + band.h}, kDivider);

  const float mid = band.y + band.h * 0.5F;
  float x = band.x + kPadX;

  if(font_heading_ != nullptr) {
    ImGui::PushFont(font_heading_);
  }
  const char* const kName = "ORB-SLAM2";
  const char* const kSub = " / Map Viewer";
  const float name_w = ImGui::CalcTextSize(kName).x;
  const float head_h = ImGui::GetTextLineHeight();
  draw->AddText({x, mid - head_h * 0.5F}, kText, kName);
  draw->AddText({x + name_w, mid - head_h * 0.5F}, kAccent700, kSub);
  x += name_w + ImGui::CalcTextSize(kSub).x + 20.0F;
  if(font_heading_ != nullptr) {
    ImGui::PopFont();
  }

  const float chip_y = mid - (ImGui::GetTextLineHeight() + 6.0F) * 0.5F;
  x += Chip(draw, {x, chip_y}, dataset_tag_.c_str(), kAccent, 0, kAccent) + 8.0F;
  Chip(draw, {x, chip_y}, vocab_tag_.c_str(), 0, kAccent100, kAccent700);

  // The segmented SLAM / Localization control, right-aligned before the state light.
  const char* const kStateText = StateLabel(snap.state);
  const float state_w = ImGui::CalcTextSize(kStateText).x + 20.0F;
  const float seg_w = ImGui::CalcTextSize("SLAM").x + ImGui::CalcTextSize("Localization").x + 4.0F * 12.0F;
  ImGui::SetCursorScreenPos({band.x + band.w - kPadX - state_w - seg_w - 20.0F, mid - 16.0F});

  const auto segment = [&](const char* label, bool active) {
    ImGui::PushStyleColor(ImGuiCol_Button, active ? kAccent : IM_COL32(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_Text, active ? kBg : kText);
    ImGui::PushStyleColor(ImGuiCol_Border, kDivider);
    const bool pressed = ImGui::Button(label, {0.0F, 32.0F});
    ImGui::PopStyleColor(3);
    return pressed;
  };
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
  if(segment("SLAM", !snap.localization_mode) && snap.localization_mode) {
    slam_.DeactivateLocalizationMode();
  }
  ImGui::SameLine();
  if(segment("Localization", snap.localization_mode) && !snap.localization_mode) {
    slam_.ActivateLocalizationMode();
  }
  ImGui::PopStyleVar();

  const float dot_x = band.x + band.w - kPadX - state_w;
  draw->AddRectFilled({dot_x, mid - 4.0F}, {dot_x + 8.0F, mid + 4.0F}, StateColor(snap.state));
  draw->AddText({dot_x + 16.0F, mid - ImGui::GetTextLineHeight() * 0.5F}, kText, kStateText);
}

void Viewer::DrawViewportOverlay(const Snapshot& snap) {
  ImDrawList* const draw = ImGui::GetWindowDrawList();
  const ImVec2 min{viewport_.x, viewport_.y};
  const ImVec2 max{viewport_.x + viewport_.w, viewport_.y + viewport_.h};
  draw->AddRect(min, max, kDivider);
  BlueprintCorners(draw, min, max, IM_COL32(0xF2, 0xF2, 0xF3, 0x8C));

  char buf[160];
  if(font_small_ != nullptr) {
    ImGui::PushFont(font_small_);
  }
  draw->AddText({min.x + 16.0F, min.y + 14.0F}, kAccent200, "MAP \xc2\xb7 WORLD FRAME");
  std::snprintf(buf,
                sizeof(buf),
                "%s \xc2\xb7 yaw %.0f\xc2\xb0 \xc2\xb7 pitch %.0f\xc2\xb0 \xc2\xb7 %.1f m",
                follow_camera_ ? "Tcw follow" : "free",
                static_cast<double>(yaw_) * 180.0 / M_PI,
                static_cast<double>(pitch_) * 180.0 / M_PI,
                static_cast<double>(distance_));
  draw->AddText({min.x + 16.0F + ImGui::CalcTextSize("MAP \xc2\xb7 WORLD FRAME").x + 12.0F, min.y + 14.0F}, kHudText, buf);

  // Count chips, right-aligned along the top edge.
  const char* const kFmt[3] = {"%zu MapPoints", "%zu KeyFrames", "%zu covis. edges"};
  const std::size_t values[3] = {snap.n_map_points, snap.n_key_frames, snap.covis_edges.size()};
  float widths[3] = {};
  float total = 0.0F;
  char chips[3][64];
  for(int i = 0; i < 3; ++i) {
    std::snprintf(chips[i], sizeof(chips[i]), kFmt[i], values[i]);
    widths[i] = ImGui::CalcTextSize(chips[i]).x + 20.0F;
    total += widths[i] + 6.0F;
  }
  float cx = max.x - 16.0F - total + 6.0F;
  for(int i = 0; i < 3; ++i) {
    Chip(draw, {cx, min.y + 11.0F}, chips[i], kHudLine, 0, kHudText);
    cx += widths[i] + 6.0F;
  }

  // Legend, bottom left. Two columns, matching the layers that can be drawn.
  struct LegendRow {
    const char* label;
    ImU32 col;
    bool line;
    float thickness;
  };
  const LegendRow rows[6] = {
      {"MapPoint (all)", IM_COL32(0x94, 0xBC, 0xE3, 0xFF), false, 1.0F},
      {"MapPoint (local map)", IM_COL32(0xF2, 0xF2, 0xF3, 0xFF), false, 1.0F},
      {"KeyFrame frustum", IM_COL32(0x94, 0xBC, 0xE3, 0xFF), true, 1.0F},
      {"Current camera Tcw", IM_COL32(0xF2, 0xF2, 0xF3, 0xFF), true, 2.0F},
      {"Covisibility graph (w >= 100)", IM_COL32(0x41, 0x61, 0x80, 0xFF), true, 1.0F},
      {"Camera trajectory", IM_COL32(0xD6, 0xEB, 0xFF, 0xFF), true, 1.0F},
  };
  const float row_h = ImGui::GetTextLineHeight() + 3.0F;
  const float legend_top = max.y - 14.0F - 3.0F * row_h;
  for(int i = 0; i < 6; ++i) {
    const float lx = min.x + 16.0F + static_cast<float>(i % 2) * 200.0F;
    const float ly = legend_top + static_cast<float>(i / 2) * row_h;
    LegendSwatch(draw, {lx, ly}, rows[i].col, rows[i].line, rows[i].thickness);
    draw->AddText({lx + 22.0F, ly}, kHudText, rows[i].label);
  }
  if(font_small_ != nullptr) {
    ImGui::PopFont();
  }

  // View controls, bottom right.
  const auto view_button = [&](const char* label, bool active) {
    ImGui::PushStyleColor(ImGuiCol_Button, active ? kAccent : IM_COL32(0x1D, 0x2D, 0x3D, 0xD8));
    ImGui::PushStyleColor(ImGuiCol_Text, active ? kBg : kHudText);
    ImGui::PushStyleColor(ImGuiCol_Border, kHudLine);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(0xD6, 0xEB, 0xFF, 0x30));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(0xD6, 0xEB, 0xFF, 0x50));
    const bool pressed = ImGui::Button(label);
    ImGui::PopStyleColor(5);
    return pressed;
  };
  const float bar_w = ImGui::CalcTextSize("Follow camera").x + ImGui::CalcTextSize("Top view").x +
      ImGui::CalcTextSize("Reset view").x + 6.0F * 10.0F + 2.0F * 8.0F;
  ImGui::SetCursorScreenPos({max.x - 16.0F - bar_w, max.y - 14.0F - ImGui::GetFrameHeight()});
  if(view_button("Follow camera", follow_camera_)) {
    follow_camera_ = !follow_camera_;
  }
  ImGui::SameLine();
  if(view_button("Top view", false)) {
    pitch_ = 1.5F;
    yaw_ = 0.0F;
    follow_camera_ = false;
  }
  ImGui::SameLine();
  if(view_button("Reset view", false)) {
    ResetView();
  }
}

void Viewer::DrawFramePanel(const Snapshot& snap) {
  ImDrawList* const draw = ImGui::GetWindowDrawList();
  const ImVec2 min = ImGui::GetCursorScreenPos();
  const ImVec2 max{min.x + ImGui::GetContentRegionAvail().x, min.y + kFrameH};
  draw->AddRectFilled(min, max, kSurface);
  draw->AddRect(min, max, kDivider);
  BlueprintCorners(draw, min, max, IM_COL32(0xF2, 0xF2, 0xF3, 0x8C));

  if(!snap.current_image.empty()) {
    cv::Mat rgb;
    cv::cvtColor(snap.current_image, rgb, snap.current_image.channels() == 1 ? cv::COLOR_GRAY2RGB : cv::COLOR_BGR2RGB);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgb.cols, rgb.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data);
    texture_width_ = rgb.cols;
    texture_height_ = rgb.rows;
  }

  if(texture_width_ > 0 && texture_height_ > 0) {
    // Fit inside the panel, leaving room for the kicker and the bottom legend.
    const float box_w = max.x - min.x - 24.0F;
    const float box_h = kFrameH - 62.0F;
    const float scale = std::min(box_w / static_cast<float>(texture_width_), box_h / static_cast<float>(texture_height_));
    const ImVec2 size{static_cast<float>(texture_width_) * scale, static_cast<float>(texture_height_) * scale};
    const ImVec2 origin{min.x + (max.x - min.x - size.x) * 0.5F, min.y + 32.0F + (box_h - size.y) * 0.5F};
    draw->AddImage(static_cast<ImTextureID>(static_cast<std::intptr_t>(texture_)), origin, {origin.x + size.x, origin.y + size.y});

    constexpr ImU32 kMatched = IM_COL32(0x94, 0xBC, 0xE3, 0xFF);
    constexpr ImU32 kVo = IM_COL32(0xD6, 0xEB, 0xFF, 0x99);
    for(std::size_t i = 0; i < snap.current_keys.size(); ++i) {
      const ImVec2 p{origin.x + snap.current_keys[i][0] * scale, origin.y + snap.current_keys[i][1] * scale};
      if(snap.current_mapped[i] != 0U) {
        draw->AddRectFilled({p.x - 2.0F, p.y - 2.0F}, {p.x + 2.0F, p.y + 2.0F}, kMatched);
      } else {
        draw->AddRect({p.x - 2.5F, p.y - 2.5F}, {p.x + 2.5F, p.y + 2.5F}, kVo);
      }
    }
  }

  char buf[128];
  if(font_small_ != nullptr) {
    ImGui::PushFont(font_small_);
  }
  std::snprintf(buf, sizeof(buf), "CURRENT FRAME \xc2\xb7 #%lu", snap.frame_id);
  draw->AddText({min.x + 12.0F, min.y + 10.0F}, kAccent700, buf);
  std::snprintf(buf,
                sizeof(buf),
                "%d\xc3\x97%d \xc2\xb7 %.1f fps",
                static_cast<int>(snap.width),
                static_cast<int>(snap.height),
                static_cast<double>(ImGui::GetIO().Framerate));
  draw->AddText({max.x - 12.0F - ImGui::CalcTextSize(buf).x, min.y + 10.0F}, kMuted, buf);

  const int vo = static_cast<int>(snap.current_keys.size()) - snap.matches_inliers;
  std::snprintf(buf, sizeof(buf), "%d matches \xc2\xb7 MapPoints", snap.matches_inliers);
  draw->AddRectFilled({min.x + 12.0F, max.y - 18.0F}, {min.x + 19.0F, max.y - 11.0F}, IM_COL32(0x94, 0xBC, 0xE3, 0xFF));
  draw->AddText({min.x + 26.0F, max.y - 20.0F}, kText, buf);
  const float half = min.x + 26.0F + ImGui::CalcTextSize(buf).x + 20.0F;
  std::snprintf(buf, sizeof(buf), "%d \xc2\xb7 unmatched keypoints", std::max(vo, 0));
  draw->AddRect({half, max.y - 18.0F}, {half + 7.0F, max.y - 11.0F}, IM_COL32(0xD6, 0xEB, 0xFF, 0xFF));
  draw->AddText({half + 14.0F, max.y - 20.0F}, kText, buf);
  if(font_small_ != nullptr) {
    ImGui::PopFont();
  }

  ImGui::Dummy({0.0F, kFrameH});
}

void Viewer::DrawStats(const Snapshot& snap) {
  if(font_small_ != nullptr) {
    ImGui::PushFont(font_small_);
  }
  ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kAccent700), "TRACKING");
  if(font_small_ != nullptr) {
    ImGui::PopFont();
  }

  char value[6][32];
  std::snprintf(value[0], sizeof(value[0]), "%d", snap.matches_inliers);
  std::snprintf(value[1], sizeof(value[1]), "%zu", snap.n_local_key_frames);
  std::snprintf(value[2], sizeof(value[2]), "%zu", snap.n_local_map_points);
  std::snprintf(value[3], sizeof(value[3]), "%.1f", snap.track_ms);
  std::snprintf(value[4], sizeof(value[4]), "%lu", snap.last_reloc_frame);
  std::snprintf(value[5], sizeof(value[5]), "%zu", snap.n_loop_closures);
  const char* const keys[6] = {"MATCHES", "LOCAL KFS", "LOCAL MPS", "TRACK TIME", "LAST RELOC", "LOOP CLOSURES"};
  const char* const units[6] = {"inliers this frame", "in local map", "projected", "ms / frame", "at frame", "detected"};

  constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame;
  if(ImGui::BeginTable("stats", 3, kFlags)) {
    for(int i = 0; i < 6; ++i) {
      if(i % 3 == 0) {
        ImGui::TableNextRow();
      }
      ImGui::TableSetColumnIndex(i % 3);
      if(font_small_ != nullptr) {
        ImGui::PushFont(font_small_);
      }
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kMuted), "%s", keys[i]);
      if(font_small_ != nullptr) {
        ImGui::PopFont();
      }
      if(font_stat_ != nullptr) {
        ImGui::PushFont(font_stat_);
      }
      ImGui::TextUnformatted(value[i]);
      if(font_stat_ != nullptr) {
        ImGui::PopFont();
      }
      if(font_small_ != nullptr) {
        ImGui::PushFont(font_small_);
      }
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kMuted), "%s", units[i]);
      if(font_small_ != nullptr) {
        ImGui::PopFont();
      }
    }
    ImGui::EndTable();
  }
}

void Viewer::DrawLayers() {
  if(font_small_ != nullptr) {
    ImGui::PushFont(font_small_);
  }
  ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kAccent700), "LAYERS");
  if(font_small_ != nullptr) {
    ImGui::PopFont();
  }

  bool* const toggles[6] = {&show_points_, &show_key_frames_, &show_covis_, &show_loop_, &show_spanning_, &show_local_};
  const char* const labels[6] = {"MapPoints", "KeyFrames", "Covisibility graph", "Loop edges", "Spanning tree", "Local map"};
  const float half = ImGui::GetContentRegionAvail().x * 0.5F;
  for(int i = 0; i < 6; ++i) {
    if(i % 2 == 1) {
      ImGui::SameLine(half);
    }
    if(ImGui::Checkbox(labels[i], toggles[i])) {
      uploaded_ = nullptr;
    }
  }

  if(ImGui::SliderFloat("frustum", &frustum_size_, 0.01F, 5.0F, "%.2f m") ||
     ImGui::SliderFloat("point size", &point_size_, 1.0F, 6.0F, "%.1f px")) {
    uploaded_ = nullptr;
  }

  // Actions, pinned to the bottom of the column.
  const float button_h = ImGui::GetFrameHeight() + 4.0F;
  const float remaining = ImGui::GetContentRegionAvail().y - button_h;
  if(remaining > 0.0F) {
    ImGui::Dummy({0.0F, remaining});
  }
  const float reset_w = ImGui::CalcTextSize("Reset").x + 24.0F;
  ImGui::PushStyleColor(ImGuiCol_Button, kAccent);
  ImGui::PushStyleColor(ImGuiCol_Text, kBg);
  ImGui::PushStyleColor(ImGuiCol_Border, kAccent);
  if(ImGui::Button("Save map \xc2\xb7 KeyFrameTrajectory.txt", {ImGui::GetContentRegionAvail().x - reset_w - 8.0F, button_h})) {
    slam_.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }
  ImGui::PopStyleColor(3);
  ImGui::SameLine();
  if(ImGui::Button("Reset", {reset_w, button_h})) {
    slam_.Reset();
    trajectory_.clear();
    last_traj_frame_ = ~0UL;
    selected_kf_ = -1;
    highlight_.clear();
    uploaded_ = nullptr;
  }
}

void Viewer::DrawFooter(const Snapshot& snap, const Rect& band) {
  ImDrawList* const draw = ImGui::GetWindowDrawList();
  draw->AddLine({band.x, band.y}, {band.x + band.w, band.y}, kDivider);

  const float norm = std::sqrt(snap.Twc[12] * snap.Twc[12] + snap.Twc[13] * snap.Twc[13] + snap.Twc[14] * snap.Twc[14]);
  char left[320];
  std::snprintf(left,
                sizeof(left),
                "t = %.4f s     |t| = %.3f m     Local map: %zu KFs \xc2\xb7 %zu MPs     Loop closures: %zu \xc2\xb7 last KF %lu",
                snap.timestamp,
                static_cast<double>(norm),
                snap.n_local_key_frames,
                snap.n_local_map_points,
                snap.n_loop_closures,
                snap.last_loop_kf);

  if(font_small_ != nullptr) {
    ImGui::PushFont(font_small_);
  }
  const float y = band.y + band.h * 0.5F - ImGui::GetTextLineHeight() * 0.5F;
  draw->AddText({band.x + kPadX, y}, kMuted, left);
  draw->AddText({band.x + band.w - kPadX - ImGui::CalcTextSize(orb_caption_.c_str()).x, y}, kMuted, orb_caption_.c_str());
  if(font_small_ != nullptr) {
    ImGui::PopFont();
  }
}

void Viewer::DrawUi(const Snapshot& snap) {
  const ImGuiViewport* const vp = ImGui::GetMainViewport();
  const ImVec2 pos = vp->WorkPos;
  const ImVec2 size = vp->WorkSize;

  // The viewport cell is computed analytically rather than read back from the
  // layout, because DrawScene needs it before ImGui has drawn anything.
  const float right_w = std::min(kRightW, std::max(200.0F, size.x - 2.0F * kPadX - kGap - 200.0F));
  viewport_ = Rect{pos.x + kPadX,
                   pos.y + kHeaderH + kPadY,
                   std::max(1.0F, size.x - 2.0F * kPadX - kGap - right_w),
                   std::max(1.0F, size.y - kHeaderH - kFooterH - 2.0F * kPadY)};

  ImGui::SetNextWindowPos(pos);
  ImGui::SetNextWindowSize(size);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
  constexpr ImGuiWindowFlags kRootFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
      ImGuiWindowFlags_NoBackground;
  ImGui::Begin("##root", nullptr, kRootFlags);
  ImGui::PopStyleVar();

  // Paint the chrome ground everywhere except the viewport cell, so the GL scene
  // rendered underneath shows through exactly there.
  ImDrawList* const draw = ImGui::GetWindowDrawList();
  draw->AddRectFilled(pos, {pos.x + size.x, viewport_.y}, kBg);
  draw->AddRectFilled({pos.x, viewport_.y + viewport_.h}, {pos.x + size.x, pos.y + size.y}, kBg);
  draw->AddRectFilled({pos.x, viewport_.y}, {viewport_.x, viewport_.y + viewport_.h}, kBg);
  draw->AddRectFilled({viewport_.x + viewport_.w, viewport_.y}, {pos.x + size.x, viewport_.y + viewport_.h}, kBg);

  DrawHeader(snap, Rect{pos.x, pos.y, size.x, kHeaderH});
  DrawViewportOverlay(snap);
  viewport_hovered_ = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && !ImGui::IsAnyItemHovered() &&
      ImGui::IsMousePosValid() && ImGui::GetMousePos().x >= viewport_.x && ImGui::GetMousePos().x <= viewport_.x + viewport_.w &&
      ImGui::GetMousePos().y >= viewport_.y && ImGui::GetMousePos().y <= viewport_.y + viewport_.h;

  ImGui::SetCursorScreenPos({viewport_.x + viewport_.w + kGap, viewport_.y});
  ImGui::BeginChild("##right", {right_w, viewport_.h}, ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
  DrawFramePanel(snap);
  ImGui::Dummy({0.0F, 8.0F});
  DrawStats(snap);
  ImGui::Dummy({0.0F, 8.0F});
  DrawLayers();
  ImGui::EndChild();

  DrawFooter(snap, Rect{pos.x, pos.y + size.y - kFooterH, size.x, kFooterH});
  ImGui::End();
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

  if(snap.pose_valid && snap.frame_id != last_traj_frame_) {
    trajectory_.emplace_back(snap.Twc[12], snap.Twc[13], snap.Twc[14]);
    last_traj_frame_ = snap.frame_id;
    uploaded_ = nullptr;
  }

  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  if(font_body_ != nullptr) {
    ImGui::PushFont(font_body_);
  }

  DrawUi(snap);
  HandleCameraInput(snap);
  PickKeyFrame(snap);
  // Hovering a frustum drives the same highlight as clicking one.
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
  if(font_body_ != nullptr) {
    ImGui::PopFont();
  }

  if(uploaded_ != static_cast<const void*>(&snap)) {
    UploadGeometry(snap);
  }

  int fb_width = 0;
  int fb_height = 0;
  int win_width = 0;
  int win_height = 0;
  glfwGetFramebufferSize(window_, &fb_width, &fb_height);
  glfwGetWindowSize(window_, &win_width, &win_height);
  const float dpi_scale = win_width > 0 ? static_cast<float>(fb_width) / static_cast<float>(win_width) : 1.0F;

  glViewport(0, 0, fb_width, fb_height);
  glClearColor(0.102F, 0.110F, 0.114F, 1.0F);    // kBg, for the chrome outside the viewport
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  DrawScene(dpi_scale, fb_height);

  glViewport(0, 0, fb_width, fb_height);
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_);
}

}    // namespace orbslam_viewer
