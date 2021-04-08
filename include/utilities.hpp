#pragma once

enum class eSensor : uint8_t { MONOCULAR = 0, STEREO = 1, RGBD = 2 };

// REMOVEME(Hussein)
inline void usleep(uint32_t miliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(miliseconds));
}

