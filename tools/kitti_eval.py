#!/usr/bin/env python3
"""KITTI odometry evaluation: official segment translation/rotation error + ATE RMSE.

Usage: kitti_eval.py <estimate.txt> <ground_truth.txt>

Both files are KITTI poses: one line per frame, 12 floats = 3x4 camera-to-world.
Stereo ORB-SLAM2 is metric, so no scale alignment is done -- an ATE that only looks
sane after scaling means the baseline (Camera.bf) is wrong.
"""
import sys
import numpy as np

LENGTHS = [100, 200, 300, 400, 500, 600, 700, 800]


def load(path):
    p = np.loadtxt(path).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(p), 1, 1))
    T[:, :3, :4] = p
    return T


def rot_angle(R):
    return np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))


def segment_errors(est, gt):
    """Official KITTI metric: relative error over sub-sequences of fixed length."""
    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt[:, :3, 3], axis=0), axis=1))])
    t_err, r_err = [], []
    for i in range(0, len(gt), 10):  # devkit steps 10 frames
        for length in LENGTHS:
            j = np.searchsorted(dist, dist[i] + length)
            if j >= len(gt):
                continue
            e = np.linalg.inv(est[i]) @ est[j]
            g = np.linalg.inv(gt[i]) @ gt[j]
            d = np.linalg.inv(g) @ e
            t_err.append(np.linalg.norm(d[:3, 3]) / length)
            r_err.append(rot_angle(d[:3, :3]) / length)
    if not t_err:
        sys.exit("trajectory shorter than the smallest segment (100 m)")
    return np.mean(t_err), np.mean(r_err)


def main(est_path, gt_path):
    est, gt = load(est_path), load(gt_path)
    if len(est) != len(gt):
        sys.exit(f"frame count mismatch: {len(est)} estimated vs {len(gt)} ground truth")
    t_err, r_err = segment_errors(est, gt)
    ate = np.sqrt(np.mean(np.sum((est[:, :3, 3] - gt[:, :3, 3]) ** 2, axis=1)))
    print(f"frames        {len(est)}")
    print(f"translation   {t_err * 100:.4f} %")
    print(f"rotation      {np.rad2deg(r_err) * 100:.6f} deg/100m")
    print(f"ATE RMSE      {ate:.4f} m")
    return t_err, ate


def selfcheck():
    """Ground truth against itself must be exactly zero."""
    gt = sys.argv[2] if len(sys.argv) > 2 else None
    t_err, ate = main(gt, gt)
    assert t_err < 1e-12 and ate < 1e-12, (t_err, ate)
    print("selfcheck OK")


if __name__ == "__main__":
    if sys.argv[1:2] == ["--selfcheck"]:
        selfcheck()
    elif len(sys.argv) != 3:
        sys.exit(__doc__)
    else:
        main(sys.argv[1], sys.argv[2])
