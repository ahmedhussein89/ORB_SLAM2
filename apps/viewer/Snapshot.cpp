#include "Snapshot.hpp"

#include <algorithm>
#include <mutex>
#include <unordered_set>

#include "orbslam/core/KeyFrame.hpp"
#include "orbslam/core/Map.hpp"
#include "orbslam/core/MapPoint.hpp"
#include "orbslam/tracking/Tracking.hpp"

namespace orbslam_viewer {
namespace {

// Twc as a column-major 4x4 for OpenGL. `Twc_mat` is 4x4 CV_32F row-major.
std::array<float, 16> ToGl(const cv::Mat& Twc_mat) {
  std::array<float, 16> out{};
  out[15] = 1.0F;
  if(Twc_mat.empty()) {
    out[0] = out[5] = out[10] = 1.0F;
    return out;
  }
  for(int r = 0; r < 4; ++r) {
    for(int c = 0; c < 4; ++c) {
      out[static_cast<std::size_t>(c * 4 + r)] = Twc_mat.at<float>(r, c);
    }
  }
  return out;
}

void PushEdge(std::vector<std::array<float, 6>>& edges, const cv::Mat& a, const cv::Mat& b) {
  edges.push_back({a.at<float>(0), a.at<float>(1), a.at<float>(2), b.at<float>(0), b.at<float>(1), b.at<float>(2)});
}

}    // namespace

std::shared_ptr<const Snapshot> BuildSnapshot(const ORB_SLAM2::System& slam, const cv::Mat& Tcw, double track_ms) {
  auto snap = std::make_shared<Snapshot>();
  snap->track_ms = track_ms;

  ORB_SLAM2::Map* const map = slam.GetMap();
  ORB_SLAM2::Tracking* const tracker = slam.GetTracker();

  // LoopClosing and Optimizer hold this while rewriting poses; without it the
  // cloud and the keyframe poses can come from two different optimizations.
  const std::scoped_lock map_lock(map->mMutexMapUpdate);

  snap->state = tracker->mState;
  snap->pose_valid = !Tcw.empty();
  if(snap->pose_valid) {
    snap->Twc = ToGl(Tcw.inv());
  } else {
    snap->Twc = ToGl(cv::Mat());
  }

  // ---- point cloud -------------------------------------------------------
  const std::vector<ORB_SLAM2::MapPoint*> points = map->GetAllMapPoints();
  const std::vector<ORB_SLAM2::MapPoint*> refs = map->GetReferenceMapPoints();
  std::unordered_set<PointId> ref_ids;
  ref_ids.reserve(refs.size());
  for(ORB_SLAM2::MapPoint* const p : refs) {
    if(p != nullptr && !p->isBad()) {
      ref_ids.insert(p->mnId);
    }
  }

  snap->point_xyz.reserve(points.size() * 3);
  snap->point_ids.reserve(points.size());
  snap->point_ref.reserve(points.size());
  for(ORB_SLAM2::MapPoint* const p : points) {
    if(p == nullptr || p->isBad()) {
      continue;
    }
    const cv::Mat pos = p->GetWorldPos();
    snap->point_xyz.push_back(pos.at<float>(0));
    snap->point_xyz.push_back(pos.at<float>(1));
    snap->point_xyz.push_back(pos.at<float>(2));
    snap->point_ids.push_back(p->mnId);
    snap->point_ref.push_back(static_cast<unsigned char>(ref_ids.count(p->mnId) != 0U ? 1 : 0));
  }
  snap->n_map_points = snap->point_ids.size();

  // ---- keyframes and the covisibility graph ------------------------------
  std::vector<ORB_SLAM2::KeyFrame*> kfs = map->GetAllKeyFrames();
  std::sort(kfs.begin(), kfs.end(), ORB_SLAM2::KeyFrame::lId);
  snap->key_frames.reserve(kfs.size());
  for(ORB_SLAM2::KeyFrame* const kf : kfs) {
    if(kf == nullptr || kf->isBad()) {
      continue;
    }
    KeyFrameView view;
    view.id = kf->mnId;
    view.Twc = ToGl(kf->GetPoseInverse());
    const cv::Mat ow = kf->GetCameraCenter();
    view.center = {ow.at<float>(0), ow.at<float>(1), ow.at<float>(2)};

    for(ORB_SLAM2::MapPoint* const p : kf->GetMapPointMatches()) {
      if(p != nullptr && !p->isBad()) {
        view.point_ids.push_back(p->mnId);
      }
    }
    view.n_tracked = static_cast<int>(view.point_ids.size());

    // Graph: covisibility (weight >= 100, as the old MapDrawer used), spanning
    // tree, loop edges -- one layer each. Every edge is drawn once by only
    // emitting it from the lower of the two ids.
    for(ORB_SLAM2::KeyFrame* const n : kf->GetCovisiblesByWeight(100)) {
      if(n != nullptr && !n->isBad() && n->mnId > kf->mnId) {
        PushEdge(snap->covis_edges, ow, n->GetCameraCenter());
      }
    }
    if(ORB_SLAM2::KeyFrame* const parent = kf->GetParent(); parent != nullptr && !parent->isBad()) {
      PushEdge(snap->spanning_edges, ow, parent->GetCameraCenter());
    }
    bool closes_loop = false;
    for(ORB_SLAM2::KeyFrame* const n : kf->GetLoopEdges()) {
      if(n != nullptr && !n->isBad()) {
        closes_loop = true;
        if(n->mnId > kf->mnId) {
          PushEdge(snap->loop_edges, ow, n->GetCameraCenter());
        }
      }
    }
    if(closes_loop) {
      ++snap->n_loop_closures;
      snap->last_loop_kf = kf->mnId;
    }

    snap->key_frames.push_back(std::move(view));
  }
  snap->n_key_frames = snap->key_frames.size();

  // ---- live frame --------------------------------------------------------
  // mImGray is a shallow assignment of the caller's buffer -- copy, don't alias.
  tracker->mImGray.copyTo(snap->current_image);
  const ORB_SLAM2::Frame& frame = tracker->mCurrentFrame;
  snap->frame_id = frame.mnId;
  snap->timestamp = frame.mTimeStamp;
  snap->sensor = tracker->mSensor;
  snap->localization_mode = tracker->mbOnlyTracking;
  snap->matches_inliers = tracker->mnMatchesInliers;
  snap->n_local_key_frames = tracker->mvpLocalKeyFrames.size();
  snap->n_local_map_points = tracker->mvpLocalMapPoints.size();
  snap->last_reloc_frame = tracker->mnLastRelocFrameId;
  snap->current_keys.reserve(frame.mvKeysUn.size());
  snap->current_mapped.reserve(frame.mvKeysUn.size());
  for(std::size_t i = 0; i < frame.mvKeysUn.size(); ++i) {
    snap->current_keys.push_back({frame.mvKeysUn[i].pt.x, frame.mvKeysUn[i].pt.y});
    ORB_SLAM2::MapPoint* const p = i < frame.mvpMapPoints.size() ? frame.mvpMapPoints[i] : nullptr;
    const bool outlier = i < frame.mvbOutlier.size() && frame.mvbOutlier[i];
    snap->current_mapped.push_back(static_cast<unsigned char>(p != nullptr && !outlier && p->Observations() > 0 ? 1 : 0));
  }
  if(!frame.mK.empty()) {
    snap->fx = frame.mK.at<float>(0, 0);
    snap->fy = frame.mK.at<float>(1, 1);
    snap->cx = frame.mK.at<float>(0, 2);
    snap->cy = frame.mK.at<float>(1, 2);
  }
  if(!snap->current_image.empty()) {
    snap->width = static_cast<float>(snap->current_image.cols);
    snap->height = static_cast<float>(snap->current_image.rows);
  }

  // ponytail: rebuilds the whole cloud every frame. A 100k-point map is ~1.2 MB and
  // a few ms; if it ever stalls tracking, diff by mnId against the previous
  // snapshot and only append the new points.
  return snap;
}

}    // namespace orbslam_viewer
