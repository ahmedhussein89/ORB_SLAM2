/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#include <mutex>
#include <vector>

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "orbslam/tracking/Tracking.hpp"
#include "orbslam/core/Map.hpp"
#include "orbslam/mapping/LocalMapping.hpp"
#include "orbslam/loop_closing/LoopClosing.hpp"
#include "orbslam/features/KeyFrameDatabase.hpp"
#include "orbslam/features/ORBVocabulary.hpp"

namespace ORB_SLAM2 {

class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System {
public:
  // Input sensor
  enum eSensor { MONOCULAR = 0, STEREO = 1, RGBD = 2 };

public:
  // Initialize the SLAM system. It launches the Local Mapping and Loop Closing threads.
  // bUseViewer is accepted but ignored: the viewer now lives in a separate GUI application.
  System(const std::string& strVocFile, const std::string& strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

  // Joins the two mapping threads by calling Shutdown() (which is idempotent).
  // The objects this class news are deliberately NOT freed here: LoopClosing can
  // leave a *detached* global-BA thread running past Shutdown(), so releasing the
  // Map or the KeyFrameDatabase under it is a use-after-free. Reclaiming them
  // means making that thread joinable first -- a behaviour change, not a refactor.
  ~System();

  // Owns two running threads, so it is neither copyable nor movable (C.21).
  System(const System&) = delete;
  System& operator=(const System&) = delete;
  System(System&&) = delete;
  System& operator=(System&&) = delete;

  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Returns the camera pose (empty if tracking fails).
  cv::Mat TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp);

  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Input depthmap: Float (CV_32F).
  // Returns the camera pose (empty if tracking fails).
  cv::Mat TrackRGBD(const cv::Mat& im, const cv::Mat& depthmap, const double& timestamp);

  // Proccess the given monocular frame
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Returns the camera pose (empty if tracking fails).
  cv::Mat TrackMonocular(const cv::Mat& im, const double& timestamp);

  // This stops local mapping thread (map building) and performs only camera tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear map)
  void Reset();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();

  // Save camera trajectory in the TUM RGB-D dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveTrajectoryTUM(const std::string& filename);

  // Save keyframe poses in the TUM RGB-D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveKeyFrameTrajectoryTUM(const std::string& filename);

  // Save camera trajectory in the KITTI dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  void SaveTrajectoryKITTI(const std::string& filename);

  // TODO: Save/Load functions
  // SaveMap(const string &filename);
  // LoadMap(const string &filename);

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  int GetTrackingState();
  std::vector<MapPoint*> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

  // Read-only handles for an external visualizer. The GUI lives in its own
  // executable (see .claude/plans/migration.md sections 15-17); these two are the
  // only door it needs. Both objects are internally mutex-protected -- take
  // Map::mMutexMapUpdate around anything that walks poses.
  Map* GetMap() const noexcept {
    return mpMap;
  }
  Tracking* GetTracker() const noexcept {
    return mpTracker;
  }

private:
  // Input sensor
  eSensor mSensor;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary* mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop detection).
  KeyFrameDatabase* mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  Map* mpMap;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints and
  // performs relocalization if tracking fails.
  Tracking* mpTracker;

  // Local Mapper. It manages the local map and performs local bundle adjustment.
  LocalMapping* mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
  // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
  LoopClosing* mpLoopCloser;

  // System threads: Local Mapping, Loop Closing.
  // The Tracking thread "lives" in the main execution thread that creates the System object.
  // Declared after the objects they run on, so they are joined (jthread's
  // destructor) before those objects are destroyed.
  std::jthread mptLocalMapping;
  std::jthread mptLoopClosing;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Tracking state
  int mTrackingState;
  std::vector<MapPoint*> mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex mMutexState;
};

}    // namespace ORB_SLAM2
