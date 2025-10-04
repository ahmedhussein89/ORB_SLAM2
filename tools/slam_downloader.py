#!/usr/bin/env python3
"""
SLAM Dataset Downloader
Downloads KITTI, TUM RGB-D, and EuRoC MAV datasets
"""

import os
import sys
import argparse
import requests
import zipfile
import tarfile
import gzip
import shutil
from urllib.parse import urljoin, urlparse
from pathlib import Path
import time
from tqdm import tqdm


class SLAMDatasetDownloader:
    def __init__(self, base_dir="datasets"):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(exist_ok=True)

        # Dataset URLs and configurations
        self.kitti_base_url = (
            "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_"
        )
        self.tum_base_url = "https://vision.in.tum.de/rgbd/dataset/freiburg"
        self.euroc_base_url = (
            "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
        )

        # Dataset configurations
        self.kitti_sequences = {
            "color": list(range(11)),  # 00-10
            "gray": list(range(22)),  # 00-21
            "velodyne": list(range(22)),
            "calib": ["calib"],
        }

        self.tum_datasets = {
            "freiburg1": [
                "xyz",
                "360",
                "floor",
                "desk",
                "desk2",
                "room",
                "plant",
                "teddy",
                "rpy",
                "calibration_rgb_camera",
                "calibration_ir_camera",
            ],
            "freiburg2": [
                "xyz",
                "360_hemisphere",
                "360_kidnap",
                "desk",
                "large_no_loop",
                "large_with_loop",
                "pioneer_360",
                "pioneer_slam",
                "pioneer_slam2",
                "pioneer_slam3",
                "rpy",
                "flowerbouquet",
                "flowerbouquet_browncouch",
                "metallic_sphere",
                "metallic_sphere2",
                "dishes",
                "coke",
            ],
            "freiburg3": [
                "xyz",
                "long_office_household",
                "nostructure_texture_near_and_far",
                "nostructure_texture_far",
                "structure_texture_near",
                "structure_notexture_near",
                "structure_notexture_far",
                "sitting_xyz",
                "sitting_halfsphere",
                "sitting_rpy",
                "walking_xyz",
                "walking_halfsphere",
                "walking_rpy",
                "walking_static",
                "cabinet",
                "large_cabinet",
                "teddy",
            ],
        }

        self.euroc_datasets = {
            "machine_hall": [
                "MH_01_easy",
                "MH_02_easy",
                "MH_03_medium",
                "MH_04_difficult",
                "MH_05_difficult",
            ],
            "vicon_room1": ["V1_01_easy", "V1_02_medium", "V1_03_difficult"],
            "vicon_room2": ["V2_01_easy", "V2_02_medium", "V2_03_difficult"],
        }

    def download_file(self, url, filepath, chunk_size=8192):
        """Download file with progress bar"""
        try:
            response = requests.get(url, stream=True)
            response.raise_for_status()

            total_size = int(response.headers.get("content-length", 0))

            with (
                open(filepath, "wb") as f,
                tqdm(
                    desc=f"Downloading {filepath.name}",
                    total=total_size,
                    unit="B",
                    unit_scale=True,
                    unit_divisor=1024,
                ) as pbar,
            ):
                for chunk in response.iter_content(chunk_size=chunk_size):
                    if chunk:
                        f.write(chunk)
                        pbar.update(len(chunk))

            return True

        except Exception as e:
            print(f"Error downloading {url}: {e}")
            return False

    def extract_archive(self, archive_path, extract_to):
        """Extract various archive formats"""
        print(f"Extracting {archive_path.name}...")

        try:
            if archive_path.suffix == ".zip":
                with zipfile.ZipFile(archive_path, "r") as zip_ref:
                    zip_ref.extractall(extract_to)
            elif (
                archive_path.suffix in [".tar", ".tgz"] or ".tar." in archive_path.name
            ):
                with tarfile.open(archive_path, "r:*") as tar_ref:
                    tar_ref.extractall(extract_to)
            elif archive_path.suffix == ".gz" and not ".tar." in archive_path.name:
                # Single gzipped file
                with gzip.open(archive_path, "rb") as f_in:
                    output_file = extract_to / archive_path.stem
                    with open(output_file, "wb") as f_out:
                        shutil.copyfileobj(f_in, f_out)
            else:
                print(f"Unsupported archive format: {archive_path}")
                return False

            print(f"Extracted to {extract_to}")
            return True

        except Exception as e:
            print(f"Error extracting {archive_path}: {e}")
            return False

    def download_kitti(self, sequences=None, data_types=None):
        """Download KITTI Odometry dataset"""
        print("=" * 50)
        print("DOWNLOADING KITTI ODOMETRY DATASET")
        print("=" * 50)

        if sequences is None:
            sequences = list(range(11))  # Default: sequences 00-10
        if data_types is None:
            data_types = ["color", "calib"]  # Default: color images and calibration

        kitti_dir = self.base_dir / "kitti"
        kitti_dir.mkdir(exist_ok=True)

        urls_to_download = []

        # Build download URLs
        for data_type in data_types:
            if data_type == "calib":
                url = f"{self.kitti_base_url}calib.zip"
                urls_to_download.append((url, f"data_odometry_calib.zip"))
            else:
                url = f"{self.kitti_base_url}{data_type}.zip"
                urls_to_download.append((url, f"data_odometry_{data_type}.zip"))

        # Download and extract
        for url, filename in urls_to_download:
            filepath = kitti_dir / filename

            if filepath.exists():
                print(f"File {filename} already exists, skipping...")
                continue

            print(f"Downloading KITTI {filename}...")
            if self.download_file(url, filepath):
                self.extract_archive(filepath, kitti_dir)
                # Optional: remove zip file after extraction
                # filepath.unlink()

        print("KITTI download completed!")
        print(f"Dataset location: {kitti_dir.absolute()}")

    def download_tum(self, datasets=None):
        """Download TUM RGB-D dataset"""
        print("=" * 50)
        print("DOWNLOADING TUM RGB-D DATASET")
        print("=" * 50)

        if datasets is None:
            # Default: download a few representative sequences
            datasets = [
                ("freiburg1", ["xyz", "desk", "room"]),
                ("freiburg2", ["xyz", "desk"]),
                ("freiburg3", ["xyz", "long_office_household"]),
            ]

        tum_dir = self.base_dir / "tum"
        tum_dir.mkdir(exist_ok=True)

        for location, sequences in datasets:
            location_dir = tum_dir / location
            location_dir.mkdir(exist_ok=True)

            for sequence in sequences:
                print(f"Downloading TUM {location}_{sequence}...")

                # TUM dataset URL structure
                url = f"{self.tum_base_url}{location[-1]}/{location}_{sequence}.tgz"
                filename = f"{location}_{sequence}.tgz"
                filepath = location_dir / filename

                if filepath.exists():
                    print(f"File {filename} already exists, skipping...")
                    continue

                if self.download_file(url, filepath):
                    self.extract_archive(filepath, location_dir)
                    # Optional: remove tgz file after extraction
                    # filepath.unlink()

        print("TUM RGB-D download completed!")
        print(f"Dataset location: {tum_dir.absolute()}")

    def download_euroc(self, datasets=None):
        """Download EuRoC MAV dataset"""
        print("=" * 50)
        print("DOWNLOADING EuRoC MAV DATASET")
        print("=" * 50)

        if datasets is None:
            # Default: download a few representative sequences
            datasets = [
                ("machine_hall", ["MH_01_easy", "MH_02_easy"]),
                ("vicon_room1", ["V1_01_easy"]),
                ("vicon_room2", ["V2_01_easy"]),
            ]

        euroc_dir = self.base_dir / "euroc"
        euroc_dir.mkdir(exist_ok=True)

        for location, sequences in datasets:
            location_dir = euroc_dir / location
            location_dir.mkdir(exist_ok=True)

            for sequence in sequences:
                print(f"Downloading EuRoC {sequence}...")

                # EuRoC dataset URL structure
                url = f"{self.euroc_base_url}/{location}/{sequence}/{sequence}.zip"
                filename = f"{sequence}.zip"
                filepath = location_dir / filename

                if filepath.exists():
                    print(f"File {filename} already exists, skipping...")
                    continue

                if self.download_file(url, filepath):
                    self.extract_archive(filepath, location_dir)
                    # Optional: remove zip file after extraction
                    # filepath.unlink()

        print("EuRoC MAV download completed!")
        print(f"Dataset location: {euroc_dir.absolute()}")

    def list_available_datasets(self):
        """List all available datasets"""
        print("=" * 60)
        print("AVAILABLE SLAM DATASETS")
        print("=" * 60)

        print("\n📁 KITTI Odometry:")
        print("   Sequences: 00-21 (00-10 have ground truth)")
        print("   Data types: color, gray, velodyne, calib")
        print("   Size: ~165GB (color), ~80GB (gray), ~400GB (velodyne)")

        print("\n📁 TUM RGB-D:")
        for location, sequences in self.tum_datasets.items():
            print(f"   {location}: {len(sequences)} sequences")
            print(f"      Examples: {sequences[:5]}...")
        print("   Total size: ~50GB")

        print("\n📁 EuRoC MAV:")
        for location, sequences in self.euroc_datasets.items():
            print(f"   {location}: {sequences}")
        print("   Total size: ~20GB")

        print("\n" + "=" * 60)

    def download_custom_urls(self, urls, output_dir):
        """Download custom list of URLs"""
        output_path = self.base_dir / output_dir
        output_path.mkdir(exist_ok=True)

        for url in urls:
            filename = Path(urlparse(url).path).name
            if not filename:
                filename = f"download_{int(time.time())}"

            filepath = output_path / filename
            print(f"Downloading {url}...")

            if self.download_file(url, filepath):
                # Try to extract if it's an archive
                if filepath.suffix in [".zip", ".tar", ".tgz", ".gz"]:
                    self.extract_archive(filepath, output_path)


def main():
    parser = argparse.ArgumentParser(
        description="Download SLAM datasets (KITTI, TUM, EuRoC)"
    )
    parser.add_argument(
        "--dataset",
        "-d",
        choices=["kitti", "tum", "euroc", "all"],
        default="all",
        help="Dataset to download",
    )
    parser.add_argument(
        "--output-dir", "-o", default="datasets", help="Output directory for datasets"
    )
    parser.add_argument(
        "--list", "-l", action="store_true", help="List available datasets"
    )
    parser.add_argument(
        "--kitti-sequences",
        nargs="+",
        type=int,
        help="KITTI sequences to download (e.g., 0 1 2)",
    )
    parser.add_argument(
        "--kitti-types",
        nargs="+",
        choices=["color", "gray", "velodyne", "calib"],
        default=["color", "calib"],
        help="KITTI data types to download",
    )
    parser.add_argument(
        "--tum-quick",
        action="store_true",
        help="Download only representative TUM sequences",
    )
    parser.add_argument(
        "--euroc-quick",
        action="store_true",
        help="Download only representative EuRoC sequences",
    )
    parser.add_argument("--urls", nargs="+", help="Custom URLs to download")

    args = parser.parse_args()

    downloader = SLAMDatasetDownloader(args.output_dir)

    if args.list:
        downloader.list_available_datasets()
        return

    if args.urls:
        downloader.download_custom_urls(args.urls, "custom")
        return

    try:
        if args.dataset in ["kitti", "all"]:
            sequences = args.kitti_sequences
            if sequences is None:
                sequences = list(range(11))  # Default: 00-10
            downloader.download_kitti(sequences=sequences, data_types=args.kitti_types)

        if args.dataset in ["tum", "all"]:
            if args.tum_quick:
                # Quick download - representative sequences
                datasets = [
                    ("freiburg1", ["xyz", "desk"]),
                    ("freiburg2", ["xyz", "desk"]),
                    ("freiburg3", ["xyz", "long_office_household"]),
                ]
            else:
                # Full download
                datasets = [
                    ("freiburg1", ["xyz", "360", "floor", "desk", "room"]),
                    ("freiburg2", ["xyz", "desk", "large_with_loop"]),
                    ("freiburg3", ["xyz", "long_office_household", "walking_xyz"]),
                ]
            downloader.download_tum(datasets)

        if args.dataset in ["euroc", "all"]:
            if args.euroc_quick:
                # Quick download - one from each location
                datasets = [
                    ("machine_hall", ["MH_01_easy"]),
                    ("vicon_room1", ["V1_01_easy"]),
                    ("vicon_room2", ["V2_01_easy"]),
                ]
            else:
                # More comprehensive download
                datasets = [
                    ("machine_hall", ["MH_01_easy", "MH_02_easy", "MH_03_medium"]),
                    ("vicon_room1", ["V1_01_easy", "V1_02_medium"]),
                    ("vicon_room2", ["V2_01_easy", "V2_02_medium"]),
                ]
            downloader.download_euroc(datasets)

    except KeyboardInterrupt:
        print("\nDownload interrupted by user")
    except Exception as e:
        print(f"Error during download: {e}")


if __name__ == "__main__":
    # Example usage if run directly
    if len(sys.argv) == 1:
        print("SLAM Dataset Downloader")
        print("=" * 30)
        print("\nUsage examples:")
        print(
            "python slam_downloader.py --list                    # List available datasets"
        )
        print("python slam_downloader.py --dataset kitti          # Download KITTI")
        print(
            "python slam_downloader.py --dataset tum --tum-quick  # Download TUM (quick)"
        )
        print("python slam_downloader.py --dataset euroc          # Download EuRoC")
        print(
            "python slam_downloader.py --dataset all            # Download all datasets"
        )
        print("\nKITTI options:")
        print("python slam_downloader.py -d kitti --kitti-sequences 0 1 2")
        print("python slam_downloader.py -d kitti --kitti-types color calib")
        print("\nCustom URLs:")
        print("python slam_downloader.py --urls http://example.com/dataset.zip")
        print("\n" + "=" * 50)

        # Show quick demo
        downloader = SLAMDatasetDownloader("demo_datasets")
        downloader.list_available_datasets()
    else:
        main()
