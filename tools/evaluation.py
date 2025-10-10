#!/usr/bin/env python3
"""
ORB-SLAM2 Trajectory Comparison Tool
Compares two KeyFrameTrajectory.txt files similar to EVO tool
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
from scipy.spatial.transform import Rotation as R
from scipy import linalg
import pandas as pd


class TrajectoryComparator:
    def __init__(self):
        self.reference_trajectory = None
        self.estimated_trajectory = None
        self.aligned_estimated = None
        self.timestamps_ref = None
        self.timestamps_est = None

    def load_keyframe_trajectory(self, filepath):
        """
        Load KeyFrameTrajectory.txt file from ORB-SLAM2
        Format: timestamp tx ty tz qx qy qz qw
        """
        try:
            data = np.loadtxt(filepath)
            timestamps = data[:, 0]
            positions = data[:, 1:4]  # tx, ty, tz
            quaternions = data[:, 4:8]  # qx, qy, qz, qw

            # Convert quaternions to rotation matrices
            rotations = []
            for quat in quaternions:
                # ORB-SLAM2 uses qx,qy,qz,qw format, scipy uses w,x,y,z
                quat_scipy = np.array([quat[3], quat[0], quat[1], quat[2]])
                rot_matrix = R.from_quat(quat_scipy).as_matrix()
                rotations.append(rot_matrix)

            rotations = np.array(rotations)

            return {
                "timestamps": timestamps,
                "positions": positions,
                "rotations": rotations,
                "quaternions": quaternions,
            }

        except Exception as e:
            print(f"Error loading trajectory file {filepath}: {e}")
            return None

    def align_trajectories(self, ref_traj, est_traj, alignment_mode="sim3"):
        """
        Align trajectories using different alignment methods
        alignment_mode: 'sim3' (similarity), 'se3' (rigid), 'scale_only'
        """
        # Find common timestamps or interpolate
        ref_positions = ref_traj["positions"]
        est_positions = est_traj["positions"]

        # For simplicity, assume trajectories have same length or take minimum
        min_len = min(len(ref_positions), len(est_positions))
        ref_positions = ref_positions[:min_len]
        est_positions = est_positions[:min_len]

        # Check if trajectories are identical (same file comparison)
        position_diff = np.abs(ref_positions - est_positions)
        max_diff = np.max(position_diff)

        if max_diff < 1e-10:  # Trajectories are essentially identical
            print("Trajectories are identical - skipping alignment")
            return est_positions

        # Check if alignment is needed (small initial error might not need alignment)
        initial_error = np.mean(np.linalg.norm(ref_positions - est_positions, axis=1))
        print(f"Initial alignment error: {initial_error:.6f} m")

        if alignment_mode == "scale_only":
            print("Applying scale-only alignment for monocular SLAM...")
            return self.scale_alignment(est_positions, ref_positions)
        elif alignment_mode == "se3":
            print("Applying SE(3) rigid alignment...")
            return self.rigid_alignment(est_positions.T, ref_positions.T).T
        else:  # sim3
            # Only align if there's significant error
            if initial_error > 0.01:  # 1cm threshold
                print("Applying Sim(3) alignment...")
                aligned_est = self.umeyama_alignment(est_positions.T, ref_positions.T)
                return aligned_est.T
            else:
                print("Trajectories are well aligned - skipping alignment")
                return est_positions

    def umeyama_alignment(self, X, Y):
        """
        Umeyama alignment between two point sets
        X: estimated trajectory points (3xN)
        Y: reference trajectory points (3xN)
        """
        # Check for degenerate case (insufficient points or identical points)
        if X.shape[1] < 3:
            print("Warning: Insufficient points for reliable alignment")
            return X

        # Center the point sets
        mu_X = X.mean(axis=1, keepdims=True)
        mu_Y = Y.mean(axis=1, keepdims=True)

        X_centered = X - mu_X
        Y_centered = Y - mu_Y

        # Check if point sets have zero variance (all points are the same)
        if np.allclose(X_centered, 0) or np.allclose(Y_centered, 0):
            print("Warning: Point set has zero variance - applying translation only")
            return X - mu_X + mu_Y

        # Compute cross-covariance matrix
        Sigma = X_centered @ Y_centered.T / X_centered.shape[1]

        # SVD
        U, D, Vt = linalg.svd(Sigma)
        V = Vt.T

        # Compute rotation
        S = np.eye(X.shape[0])
        if linalg.det(U) * linalg.det(V) < 0:
            S[-1, -1] = -1

        R_opt = U @ S @ V.T

        # Compute scale
        trace_X = np.trace(X_centered @ X_centered.T)
        trace_D_S = np.trace(np.diag(D) @ S)

        if trace_X < 1e-10:  # Avoid division by very small number
            scale = 1.0
        else:
            scale = trace_D_S / trace_X

        # Ensure scale is reasonable (between 0.1 and 10)
        scale = np.clip(scale, 0.1, 10.0)

        # Compute translation
        t_opt = mu_Y - scale * R_opt @ mu_X

        # Apply transformation
        X_aligned = scale * R_opt @ X + t_opt

        return X_aligned

    def scale_alignment(self, est_positions, ref_positions):
        """
        Align trajectories using only scale factor (for monocular SLAM)
        This finds the best scale factor to match trajectory lengths
        """
        # Method 1: Scale based on total trajectory length
        est_distances = np.linalg.norm(np.diff(est_positions, axis=0), axis=1)
        ref_distances = np.linalg.norm(np.diff(ref_positions, axis=0), axis=1)

        est_total_length = np.sum(est_distances)
        ref_total_length = np.sum(ref_distances)

        if est_total_length < 1e-6:
            print("Warning: Estimated trajectory has zero length")
            return est_positions

        scale_factor = ref_total_length / est_total_length
        print(f"Scale factor (trajectory length): {scale_factor:.4f}")

        # Apply scale from the first position (keep starting point fixed)
        start_pos = est_positions[0]
        scaled_positions = start_pos + scale_factor * (est_positions - start_pos)

        # Method 2: Alternative - scale based on end-to-end distance
        est_span = np.linalg.norm(est_positions[-1] - est_positions[0])
        ref_span = np.linalg.norm(ref_positions[-1] - ref_positions[0])

        if est_span > 1e-6:
            scale_factor_span = ref_span / est_span
            print(f"Scale factor (end-to-end): {scale_factor_span:.4f}")

        return scaled_positions

    def rigid_alignment(self, X, Y):
        """
        SE(3) rigid alignment (rotation + translation, no scale)
        X: estimated trajectory points (3xN)
        Y: reference trajectory points (3xN)
        """
        # Center the point sets
        mu_X = X.mean(axis=1, keepdims=True)
        mu_Y = Y.mean(axis=1, keepdims=True)

        X_centered = X - mu_X
        Y_centered = Y - mu_Y

        # Compute cross-covariance matrix
        H = X_centered @ Y_centered.T

        # SVD
        U, D, Vt = linalg.svd(H)
        V = Vt.T

        # Compute rotation
        R = V @ U.T

        # Ensure proper rotation matrix (det = 1)
        if linalg.det(R) < 0:
            V[:, -1] *= -1
            R = V @ U.T

        # Compute translation
        t = mu_Y - R @ mu_X

        # Apply transformation
        X_aligned = R @ X + t

        return X_aligned

    def compute_ate(self, ref_positions, est_positions):
        """
        Compute Absolute Trajectory Error (ATE)
        """
        errors = np.linalg.norm(ref_positions - est_positions, axis=1)

        ate_stats = {
            "rmse": np.sqrt(np.mean(errors**2)),
            "mean": np.mean(errors),
            "median": np.median(errors),
            "std": np.std(errors),
            "min": np.min(errors),
            "max": np.max(errors),
            "errors": errors,
        }

        return ate_stats

    def compute_rpe(self, ref_positions, est_positions, delta=1):
        """
        Compute Relative Pose Error (RPE)
        """
        ref_relative = []
        est_relative = []

        for i in range(len(ref_positions) - delta):
            ref_rel = ref_positions[i + delta] - ref_positions[i]
            est_rel = est_positions[i + delta] - est_positions[i]

            ref_relative.append(np.linalg.norm(ref_rel))
            est_relative.append(np.linalg.norm(est_rel))

        ref_relative = np.array(ref_relative)
        est_relative = np.array(est_relative)

        errors = np.abs(ref_relative - est_relative)

        rpe_stats = {
            "rmse": np.sqrt(np.mean(errors**2)),
            "mean": np.mean(errors),
            "median": np.median(errors),
            "std": np.std(errors),
            "min": np.min(errors),
            "max": np.max(errors),
            "errors": errors,
        }

        return rpe_stats

    def print_results(self, ate_stats, rpe_stats):
        """
        Print results in EVO-like format
        """
        print("=" * 60)
        print("TRAJECTORY COMPARISON RESULTS")
        print("=" * 60)

        print("\nAbsolute Trajectory Error (ATE) [m]:")
        print("-" * 40)
        print(f"RMSE:     {ate_stats['rmse']:.6f}")
        print(f"Mean:     {ate_stats['mean']:.6f}")
        print(f"Median:   {ate_stats['median']:.6f}")
        print(f"Std:      {ate_stats['std']:.6f}")
        print(f"Min:      {ate_stats['min']:.6f}")
        print(f"Max:      {ate_stats['max']:.6f}")

        print(f"\nRelative Pose Error (RPE) [m]:")
        print("-" * 40)
        print(f"RMSE:     {rpe_stats['rmse']:.6f}")
        print(f"Mean:     {rpe_stats['mean']:.6f}")
        print(f"Median:   {rpe_stats['median']:.6f}")
        print(f"Std:      {rpe_stats['std']:.6f}")
        print(f"Min:      {rpe_stats['min']:.6f}")
        print(f"Max:      {rpe_stats['max']:.6f}")

        print("=" * 60)

    def plot_trajectories(self, save_path=None):
        """
        Plot trajectories and error visualization
        """
        fig = plt.figure(figsize=(15, 12))

        # 3D trajectory plot
        ax1 = fig.add_subplot(2, 2, 1, projection="3d")
        ref_pos = self.reference_trajectory["positions"]
        est_pos = self.estimated_trajectory["positions"]
        aligned_pos = self.aligned_estimated

        min_len = min(len(ref_pos), len(aligned_pos))
        ref_pos = ref_pos[:min_len]
        aligned_pos = aligned_pos[:min_len]

        ax1.plot(
            ref_pos[:, 0],
            ref_pos[:, 1],
            ref_pos[:, 2],
            "b-",
            label="Reference",
            linewidth=2,
        )
        ax1.plot(
            aligned_pos[:, 0],
            aligned_pos[:, 1],
            aligned_pos[:, 2],
            "r--",
            label="Estimated (aligned)",
            linewidth=2,
        )

        ax1.set_xlabel("X [m]")
        ax1.set_ylabel("Y [m]")
        ax1.set_zlabel("Z [m]")
        ax1.set_title("3D Trajectories")
        ax1.legend()

        # XY plot
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(ref_pos[:, 0], ref_pos[:, 1], "b-", label="Reference", linewidth=2)
        ax2.plot(
            aligned_pos[:, 0], aligned_pos[:, 1], "r--", label="Estimated", linewidth=2
        )
        ax2.set_xlabel("X [m]")
        ax2.set_ylabel("Y [m]")
        ax2.set_title("XY Trajectory")
        ax2.legend()
        ax2.grid(True)
        ax2.axis("equal")

        # Error over time
        ax3 = fig.add_subplot(2, 2, 3)
        errors = np.linalg.norm(ref_pos - aligned_pos, axis=1)
        timestamps = self.reference_trajectory["timestamps"][:min_len]

        ax3.plot(timestamps, errors, "g-", linewidth=1.5)
        ax3.set_xlabel("Timestamp [s]")
        ax3.set_ylabel("ATE [m]")
        ax3.set_title("Absolute Trajectory Error over Time")
        ax3.grid(True)

        # Error histogram
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.hist(errors, bins=50, alpha=0.7, color="orange")
        ax4.set_xlabel("ATE [m]")
        ax4.set_ylabel("Frequency")
        ax4.set_title("ATE Distribution")
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"Plot saved to: {save_path}")

        plt.show()

    def compare_trajectories(
        self,
        ref_file,
        est_file,
        plot=True,
        save_plot=None,
        skip_alignment=False,
        alignment_mode="scale_only",
    ):
        """
        Main comparison function
        alignment_mode: 'sim3', 'se3', 'scale_only'
        """
        print(f"Loading reference trajectory: {ref_file}")
        self.reference_trajectory = self.load_keyframe_trajectory(ref_file)

        print(f"Loading estimated trajectory: {est_file}")
        self.estimated_trajectory = self.load_keyframe_trajectory(est_file)

        if self.reference_trajectory is None or self.estimated_trajectory is None:
            print("Error: Could not load one or both trajectory files")
            return

        # Check if comparing same file
        same_file = os.path.abspath(ref_file) == os.path.abspath(est_file)
        if same_file:
            print("Comparing trajectory to itself - skipping alignment")
            skip_alignment = True

        if skip_alignment:
            print("Skipping alignment step...")
            min_len = min(
                len(self.reference_trajectory["positions"]),
                len(self.estimated_trajectory["positions"]),
            )
            self.aligned_estimated = self.estimated_trajectory["positions"][:min_len]
        else:
            print(f"Aligning trajectories using {alignment_mode} alignment...")
            self.aligned_estimated = self.align_trajectories(
                self.reference_trajectory, self.estimated_trajectory, alignment_mode
            )

        # Compute metrics
        min_len = min(
            len(self.reference_trajectory["positions"]), len(self.aligned_estimated)
        )

        ref_positions = self.reference_trajectory["positions"][:min_len]
        est_positions = self.aligned_estimated[:min_len]

        print("Computing ATE...")
        ate_stats = self.compute_ate(ref_positions, est_positions)

        print("Computing RPE...")
        rpe_stats = self.compute_rpe(ref_positions, est_positions)

        # Print results
        self.print_results(ate_stats, rpe_stats)

        # Plot results
        if plot and not same_file:  # Skip plotting if comparing to itself
            self.plot_trajectories(save_plot)
        elif plot and same_file:
            print("Skipping plot for identical trajectory comparison")

        return ate_stats, rpe_stats


def main():
    parser = argparse.ArgumentParser(
        description="Compare ORB-SLAM2 KeyFrame trajectories"
    )
    parser.add_argument("reference", help="Reference trajectory file (ground truth)")
    parser.add_argument("estimated", help="Estimated trajectory file")
    parser.add_argument("--plot", action="store_true", help="Show trajectory plots")
    parser.add_argument("--save-plot", type=str, help="Save plot to file")
    parser.add_argument(
        "--no-align", action="store_true", help="Skip trajectory alignment"
    )
    parser.add_argument("--align-mode", type=str, default="scale_only", help="scale_only, sim3, se3")

    args = parser.parse_args()

    if not os.path.exists(args.reference):
        print(f"Error: Reference file {args.reference} not found")
        return

    if not os.path.exists(args.estimated):
        print(f"Error: Estimated file {args.estimated} not found")
        return

    comparator = TrajectoryComparator()
    comparator.compare_trajectories(
        args.reference, args.estimated, plot=args.plot, save_plot=args.save_plot, alignment_mode=args.align_mode
    )


if __name__ == "__main__":
    # Example usage if run directly
    import sys

    if len(sys.argv) < 3:
        print(
            "Usage: python trajectory_comparison.py reference.txt estimated.txt [--plot] [--save-plot output.png]"
        )
        print("\nExample:")
        print(
            "python trajectory_comparison.py gt_trajectory.txt keyframe_trajectory.txt --plot"
        )

        # Demo with synthetic data
        print("\nRunning demo with synthetic data...")

        # Create synthetic trajectories for demonstration
        t = np.linspace(0, 10, 100)

        # Reference trajectory (circular)
        ref_x = 2 * np.cos(t)
        ref_y = 2 * np.sin(t)
        ref_z = 0.1 * t

        # Estimated trajectory (with some noise and drift)
        est_x = 2 * np.cos(t) + 0.1 * np.random.randn(100) + 0.05 * t
        est_y = 2 * np.sin(t) + 0.1 * np.random.randn(100) + 0.02 * t
        est_z = 0.1 * t + 0.05 * np.random.randn(100)

        # Create synthetic KeyFrameTrajectory.txt format data
        ref_data = np.column_stack(
            [
                t,
                ref_x,
                ref_y,
                ref_z,
                np.zeros(100),
                np.zeros(100),
                np.zeros(100),
                np.ones(100),
            ]
        )
        est_data = np.column_stack(
            [
                t,
                est_x,
                est_y,
                est_z,
                np.zeros(100),
                np.zeros(100),
                np.zeros(100),
                np.ones(100),
            ]
        )

        np.savetxt("demo_reference.txt", ref_data, fmt="%.6f")
        np.savetxt("demo_estimated.txt", est_data, fmt="%.6f")

        comparator = TrajectoryComparator()
        comparator.compare_trajectories(
            "demo_reference.txt", "demo_estimated.txt", plot=True
        )

        # Cleanup
        os.remove("demo_reference.txt")
        os.remove("demo_estimated.txt")
    else:
        main()
