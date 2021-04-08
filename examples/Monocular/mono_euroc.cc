/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
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
// STL
#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>
// OpenCV
#include <opencv2/opencv.hpp>
// Internal
#include <ORBVocabulary.hpp>
#include <System.hpp>

static void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                       std::vector<std::string> &vstrImages,
                       std::vector<double> &vTimeStamps) {
  std::ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while (!fTimes.eof()) {
    std::string s;
    getline(fTimes, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
      double t;
      ss >> t;
      vTimeStamps.push_back(t / 1e9);
    }
  }
}

int main(int argc, char **argv) {
  if (argc != 5) {
    cerr << endl
         << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
            "path_to_image_folder path_to_times_file"
         << endl;
    return 1;
  }

  // Retrieve paths to images
  std::vector<std::string> vstrImageFilenames;
  std::vector<double> vTimestamps;
  LoadImages(std::string(argv[3]), std::string(argv[4]), vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  if (nImages <= 0) {
    std::cerr << "ERROR: Failed to load images" << std::endl;
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], eSensor::MONOCULAR, true);

  // Vector for tracking time statistics
  std::vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  std::cout << std::endl << "-------" << std::endl;
  std::cout << "Start processing sequence ..." << std::endl;
  std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

  // Main loop
  cv::Mat im;
  for (int ni = 0; ni < nImages; ni++) {
    // Read image from file
    im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];

    if (im.empty()) {
      std::cerr << std::endl
           << "Failed to load image at: " << vstrImageFilenames[ni] << std::endl;
      return 1;
    }

    auto t1 = std::chrono::steady_clock::now();
    // Pass the image to the SLAM system
    SLAM.TrackMonocular(im, tframe);
    auto t2 = std::chrono::steady_clock::now();

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1) {
      T = vTimestamps[ni + 1] - tframe;
    } else if (ni > 0) {
      T = tframe - vTimestamps[ni - 1];
    }

    if (ttrack < T) {
      usleep((T - ttrack) * 1e6);
    }
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  std::cout << "-------" << std::endl << std::endl;
  std::cout << "median tracking time: " << vTimesTrack[nImages / 2] << std::endl;
  std::cout << "mean tracking time: " << totaltime / nImages << std::endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

