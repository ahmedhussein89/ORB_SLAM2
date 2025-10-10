#!/usr/bin/env python3
"""
ORB-SLAM2 Benchmark Runner with System Monitoring

This script runs ORB-SLAM2 examples while monitoring system resources.
Designed to run as a Docker container entry point.

Requirements:
- psutil: pip install psutil
- matplotlib: pip install matplotlib
- numpy: pip install numpy

Author: Refactored for production use
"""

import os
import sys
import time
import signal
import subprocess
import threading
import json
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, Optional, Any, List
from contextlib import contextmanager
import logging

# Check required packages
try:
    import psutil
    import numpy as np
    import matplotlib

    matplotlib.use("Agg")  # Use non-interactive backend for Docker
    import matplotlib.pyplot as plt
except ImportError as e:
    print(f"ERROR: Missing required package: {e}", file=sys.stderr)
    print("Install with: pip install psutil numpy matplotlib", file=sys.stderr)
    sys.exit(1)


# Global shutdown flag for graceful termination
shutdown_requested = False


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    global shutdown_requested
    signal_name = signal.Signals(signum).name
    logger.warning(f"Received signal {signal_name}, initiating graceful shutdown...")
    shutdown_requested = True


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
    ],
)
logger = logging.getLogger(__name__)


class ConfigurationError(Exception):
    """Raised when configuration is invalid"""

    pass


class SystemMonitor:
    """Monitor system resources during ORB-SLAM2 execution"""

    def __init__(self, pid: int, interval: float = 0.1):
        """
        Initialize system monitor.

        Args:
            pid: Process ID to monitor
            interval: Sampling interval in seconds
        """
        self.pid = pid
        self.interval = interval
        self.monitoring = False
        self.data = {
            "timestamps": [],
            "cpu_percent": [],
            "memory_mb": [],
            "memory_percent": [],
        }
        self.process = None
        self.monitor_thread = None

    def start_monitoring(self) -> bool:
        """
        Start monitoring system resources.

        Returns:
            True if monitoring started successfully, False otherwise
        """
        self.monitoring = True
        try:
            self.process = psutil.Process(self.pid)
            self.monitor_thread = threading.Thread(
                target=self._monitor_loop, daemon=True
            )
            self.monitor_thread.start()
            logger.info(f"Started monitoring process {self.pid}")
            return True
        except psutil.NoSuchProcess:
            logger.error(f"Process {self.pid} not found")
            return False

    def stop_monitoring(self) -> Dict[str, List[float]]:
        """
        Stop monitoring and return collected data.

        Returns:
            Dictionary containing monitoring data
        """
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)

        logger.info(
            f"Monitoring stopped. Collected {len(self.data['timestamps'])} samples"
        )
        return self.data

    def _monitor_loop(self):
        """Main monitoring loop running in separate thread"""
        start_time = time.time()

        while self.monitoring and not shutdown_requested:
            try:
                if self.process and self.process.is_running():
                    # Get CPU and memory usage
                    cpu_percent = self.process.cpu_percent()
                    memory_info = self.process.memory_info()
                    memory_mb = memory_info.rss / (1024 * 1024)  # Convert to MB
                    memory_percent = self.process.memory_percent()

                    # Record data
                    current_time = time.time() - start_time
                    self.data["timestamps"].append(current_time)
                    self.data["cpu_percent"].append(cpu_percent)
                    self.data["memory_mb"].append(memory_mb)
                    self.data["memory_percent"].append(memory_percent)
                else:
                    break

            except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                logger.debug(f"Monitoring loop terminated: {e}")
                break
            except Exception as e:
                logger.error(f"Error in monitoring loop: {e}")
                break

            time.sleep(self.interval)


class ORBSLAM2Runner:
    """Run ORB-SLAM2 examples with monitoring and evaluation"""

    # Default configuration paths (can be overridden by environment variables)
    DEFAULT_ORBSLAM_ROOT = "/opt/orb_slam2"

    # Settings file mapping for different datasets
    SETTINGS_MAP = {
        "mono_tum": "TUM1.yaml",
        "rgbd_tum": "TUM1.yaml",
        "mono_kitti": "KITTI00-02.yaml",
        "stereo_kitti": "KITTI00-02.yaml",
        "mono_euroc": "EuRoC.yaml",
        "stereo_euroc": "EuRoC.yaml",
    }

    def __init__(
        self, output_dir: str = "benchmark_results", orbslam_root: Optional[str] = None
    ):
        """
        Initialize ORB-SLAM2 runner.

        Args:
            output_dir: Directory to save benchmark results
            orbslam_root: Root directory of ORB-SLAM2 installation
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Get ORB-SLAM2 root from environment or use default
        self.orbslam_root = Path(
            orbslam_root or os.getenv("ORBSLAM2_ROOT", self.DEFAULT_ORBSLAM_ROOT)
        )

        if not self.orbslam_root.exists():
            raise ConfigurationError(
                f"ORB-SLAM2 root directory not found: {self.orbslam_root}"
            )

        logger.info(f"ORB-SLAM2 root: {self.orbslam_root}")
        logger.info(f"Output directory: {self.output_dir}")

        # Define example configurations
        self.examples = {
            "mono_tum": {
                "executable": "mono_tum",
                "args_template": ["{vocab_path}", "{settings_path}", "{dataset_path}"],
            },
            "mono_kitti": {
                "executable": "mono_kitti",
                "args_template": ["{vocab_path}", "{settings_path}", "{dataset_path}"],
            },
            "mono_euroc": {
                "executable": "mono_euroc",
                "args_template": [
                    "{vocab_path}",
                    "{settings_path}",
                    "{dataset_path}",
                    "{time_path}",
                ],
            },
            "rgbd_tum": {
                "executable": "rgbd_tum",
                "args_template": [
                    "{vocab_path}",
                    "{settings_path}",
                    "{dataset_path}",
                    "{associations_path}",
                ],
            },
            "stereo_kitti": {
                "executable": "stereo_kitti",
                "args_template": ["{vocab_path}", "{settings_path}", "{dataset_path}"],
            },
            "stereo_euroc": {
                "executable": "stereo_euroc",
                "args_template": [
                    "{vocab_path}",
                    "{settings_path}",
                    "{dataset_path}",
                    "{time_path}",
                ],
            },
        }

    def get_vocabulary_path(self) -> Path:
        """Get path to ORB vocabulary file"""
        vocab_path = self.orbslam_root / "Vocabulary" / "ORBvoc.txt"
        if not vocab_path.exists():
            raise ConfigurationError(f"Vocabulary file not found: {vocab_path}")
        return vocab_path

    def get_settings_path(self, example_name: str) -> Path:
        """
        Get settings file path for the given example.

        Args:
            example_name: Name of the example

        Returns:
            Path to settings file
        """
        if example_name not in self.SETTINGS_MAP:
            raise ConfigurationError(f"Unknown example: {example_name}")

        settings_file = self.SETTINGS_MAP[example_name]

        # Determine subdirectory based on example type
        if "mono" in example_name:
            subdir = "Monocular"
        elif "rgbd" in example_name:
            subdir = "RGB-D"
        elif "stereo" in example_name:
            subdir = "Stereo"
        else:
            raise ConfigurationError(
                f"Cannot determine settings subdirectory for {example_name}"
            )

        settings_path = self.orbslam_root / "examples" / subdir / settings_file

        if not settings_path.exists():
            raise ConfigurationError(f"Settings file not found: {settings_path}")

        return settings_path

    def get_timestamps_path(self, settings_name: str) -> Path:
        """
        Get timestamps file path for EuRoC dataset.

        Args:
            settings_name: Settings name (e.g., MH01)

        Returns:
            Path to timestamps file
        """
        time_path = (
            self.orbslam_root
            / "examples"
            / "Monocular"
            / "EuRoC_TimeStamps"
            / f"{settings_name}.txt"
        )

        if not time_path.exists():
            raise ConfigurationError(f"Timestamps file not found: {time_path}")

        return time_path

    def validate_paths(
        self,
        executable_path: Path,
        vocab_path: Path,
        settings_path: Path,
        dataset_path: Path,
        associations_path: Optional[Path] = None,
        time_path: Optional[Path] = None,
    ):
        """
        Validate all required paths exist.

        Args:
            executable_path: Path to ORB-SLAM2 executable
            vocab_path: Path to vocabulary file
            settings_path: Path to settings file
            dataset_path: Path to dataset
            associations_path: Optional path to associations file
            time_path: Optional path to timestamps file

        Raises:
            ConfigurationError: If any required path doesn't exist
        """
        if not executable_path.exists():
            raise ConfigurationError(f"Executable not found: {executable_path}")

        if not vocab_path.exists():
            raise ConfigurationError(f"Vocabulary file not found: {vocab_path}")

        if not settings_path.exists():
            raise ConfigurationError(f"Settings file not found: {settings_path}")

        if not dataset_path.exists():
            raise ConfigurationError(f"Dataset path not found: {dataset_path}")

        if associations_path and not associations_path.exists():
            raise ConfigurationError(
                f"Associations file not found: {associations_path}"
            )

        if time_path and not time_path.exists():
            raise ConfigurationError(f"Timestamps file not found: {time_path}")

    @contextmanager
    def monitored_process(self, args: List[str], monitor_interval: float = 0.1):
        """
        Context manager for running and monitoring a process.

        Args:
            args: Command and arguments to execute
            monitor_interval: Monitoring sampling interval

        Yields:
            Tuple of (process, monitor)
        """
        process = None
        monitor = None

        try:
            process = subprocess.Popen(
                args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1
            )

            logger.info(f"Started process with PID: {process.pid}")

            monitor = SystemMonitor(process.pid, monitor_interval)
            monitor.start_monitoring()

            yield process, monitor

        finally:
            if monitor:
                monitor.stop_monitoring()

            if process and process.poll() is None:
                logger.info("Terminating process...")
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    logger.warning("Process did not terminate gracefully, killing...")
                    process.kill()
                    process.wait()

    def run_example(
        self,
        example_name: str,
        dataset_path: str,
        associations_path: Optional[str] = None,
        time_path: Optional[str] = None,
        vocab_path: Optional[str] = None,
        settings_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Run a specific ORB-SLAM2 example with monitoring.

        Args:
            example_name: Name of the example to run
            dataset_path: Path to dataset
            associations_path: Optional path to associations file (for RGB-D)
            time_path: Optional path to timestamps file (for EuRoC)
            vocab_path: Optional path to vocabulary file (auto-detected if not provided)
            settings_path: Optional path to settings file (auto-detected if not provided)

        Returns:
            Dictionary containing benchmark results
        """
        logger.info(f"Running example: {example_name}")

        if example_name not in self.examples:
            raise ValueError(
                f"Unknown example: {example_name}. Available: {list(self.examples.keys())}"
            )

        config = self.examples[example_name]

        # Get paths
        executable_path = self.orbslam_root / "bin" / config["executable"]
        vocab_path = Path(vocab_path) if vocab_path else self.get_vocabulary_path()
        settings_path = (
            Path(settings_path)
            if settings_path
            else self.get_settings_path(example_name)
        )
        dataset_path = Path(dataset_path)

        # Optional paths
        associations_path = Path(associations_path) if associations_path else None
        time_path = Path(time_path) if time_path else None

        # Validate all paths
        self.validate_paths(
            executable_path,
            vocab_path,
            settings_path,
            dataset_path,
            associations_path,
            time_path,
        )

        # Prepare arguments
        args = [str(executable_path)]
        for arg_template in config["args_template"]:
            if arg_template == "{vocab_path}":
                args.append(str(vocab_path))
            elif arg_template == "{settings_path}":
                args.append(str(settings_path))
            elif arg_template == "{dataset_path}":
                args.append(str(dataset_path))
            elif arg_template == "{time_path}":
                if time_path:
                    args.append(str(time_path))
                else:
                    raise ConfigurationError(
                        f"{example_name} requires time_path parameter"
                    )
            elif arg_template == "{associations_path}":
                if associations_path:
                    args.append(str(associations_path))
                else:
                    raise ConfigurationError(
                        f"{example_name} requires associations_path parameter"
                    )

        logger.info(f"Command: {' '.join(args)}")

        # Create output directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = self.output_dir / f"{example_name}_{timestamp}"
        run_dir.mkdir(parents=True, exist_ok=True)

        stdout_file = run_dir / "stdout.log"
        stderr_file = run_dir / "stderr.log"

        # Run process with monitoring
        start_time = time.time()
        return_code = -1
        stdout_data = b""
        stderr_data = b""
        monitor_data = {}

        try:
            with self.monitored_process(args) as (process, monitor):
                # Wait for process to complete
                try:
                    stdout_data, stderr_data = process.communicate(timeout=None)
                    return_code = process.returncode
                except KeyboardInterrupt:
                    logger.warning("Process interrupted by user")
                    process.terminate()
                    stdout_data, stderr_data = process.communicate(timeout=5)
                    return_code = -1
                except Exception as e:
                    logger.error(f"Error during process execution: {e}")
                    process.terminate()
                    try:
                        stdout_data, stderr_data = process.communicate(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        stdout_data, stderr_data = process.communicate()
                    return_code = -1

                # Get monitoring data
                monitor_data = monitor.stop_monitoring()

        except Exception as e:
            logger.error(f"Failed to run process: {e}")
            return_code = -1

        end_time = time.time()
        duration = end_time - start_time

        logger.info(f"Process finished with return code: {return_code}")
        logger.info(f"Duration: {duration:.2f} seconds")

        # Save outputs
        try:
            stdout_file.write_text(stdout_data.decode("utf-8", errors="replace"))
            stderr_file.write_text(stderr_data.decode("utf-8", errors="replace"))
        except Exception as e:
            logger.error(f"Error saving output files: {e}")

        # Calculate statistics
        cpu_stats = {}
        memory_stats = {}

        if monitor_data.get("cpu_percent"):
            cpu_stats = {
                "mean": float(np.mean(monitor_data["cpu_percent"])),
                "max": float(np.max(monitor_data["cpu_percent"])),
                "min": float(np.min(monitor_data["cpu_percent"])),
                "std": float(np.std(monitor_data["cpu_percent"])),
            }
            memory_stats = {
                "mean_mb": float(np.mean(monitor_data["memory_mb"])),
                "max_mb": float(np.max(monitor_data["memory_mb"])),
                "min_mb": float(np.min(monitor_data["memory_mb"])),
                "std_mb": float(np.std(monitor_data["memory_mb"])),
                "mean_percent": float(np.mean(monitor_data["memory_percent"])),
                "max_percent": float(np.max(monitor_data["memory_percent"])),
            }

        # Create plots
        try:
            self._create_monitoring_plots(monitor_data, run_dir)
        except Exception as e:
            logger.error(f"Error creating plots: {e}")

        # Prepare result
        result = {
            "example_name": example_name,
            "timestamp": timestamp,
            "duration_seconds": duration,
            "return_code": return_code,
            "cpu_stats": cpu_stats,
            "memory_stats": memory_stats,
            "output_dir": str(run_dir),
            "stdout_file": str(stdout_file),
            "stderr_file": str(stderr_file),
        }

        # Save result summary
        try:
            json_result = result.copy()
            json_result["monitoring_samples"] = {
                "count": len(monitor_data.get("timestamps", [])),
                "sample_rate": len(monitor_data.get("timestamps", [])) / duration
                if duration > 0
                else 0,
            }

            with open(run_dir / "benchmark_result.json", "w") as f:
                json.dump(json_result, f, indent=2)
        except Exception as e:
            logger.error(f"Error saving result summary: {e}")

        logger.info(f"Results saved to: {run_dir}")

        return result

    def _create_monitoring_plots(
        self, monitor_data: Dict[str, List[float]], output_dir: Path
    ):
        """
        Create monitoring plots.

        Args:
            monitor_data: Dictionary containing monitoring data
            output_dir: Directory to save plots
        """
        if not monitor_data.get("timestamps"):
            logger.warning("No monitoring data available for plotting")
            return

        try:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

            # CPU usage plot
            ax1.plot(
                monitor_data["timestamps"], monitor_data["cpu_percent"], linewidth=1.5
            )
            ax1.set_xlabel("Time (seconds)")
            ax1.set_ylabel("CPU Usage (%)")
            ax1.set_title("CPU Usage Over Time")
            ax1.grid(True, alpha=0.3)
            ax1.set_ylim(bottom=0)

            # Memory usage plot
            ax2.plot(
                monitor_data["timestamps"],
                monitor_data["memory_mb"],
                linewidth=1.5,
                color="orange",
            )
            ax2.set_xlabel("Time (seconds)")
            ax2.set_ylabel("Memory Usage (MB)")
            ax2.set_title("Memory Usage Over Time")
            ax2.grid(True, alpha=0.3)
            ax2.set_ylim(bottom=0)

            plt.tight_layout()
            plot_path = output_dir / "monitoring_plots.png"
            plt.savefig(str(plot_path), dpi=300, bbox_inches="tight")
            plt.close()

            logger.info(f"Monitoring plots saved to: {plot_path}")

        except Exception as e:
            logger.error(f"Error creating monitoring plots: {e}")
            plt.close("all")


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="ORB-SLAM2 Benchmark Runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run monocular TUM example
  %(prog)s --example mono_tum --dataset /data/rgbd_dataset_freiburg1_xyz
  
  # Run RGB-D TUM example with associations
  %(prog)s --example rgbd_tum --dataset /data/rgbd_dataset_freiburg1_xyz \\
           --associations /data/rgbd_dataset_freiburg1_xyz/associations.txt
  
  # Run stereo EuRoC example
  %(prog)s --example stereo_euroc --dataset /data/MH01 --settings MH01
  
Environment Variables:
  ORBSLAM2_ROOT    Path to ORB-SLAM2 installation (default: /opt/orb_slam2)
        """,
    )

    parser.add_argument(
        "--example",
        required=True,
        choices=[
            "mono_tum",
            "mono_kitti",
            "mono_euroc",
            "rgbd_tum",
            "stereo_kitti",
            "stereo_euroc",
        ],
        help="ORB-SLAM2 example to run",
    )

    parser.add_argument(
        "--dataset",
        required=True,
        help="Path to dataset directory",
    )

    parser.add_argument(
        "--settings",
        help="Settings name for EuRoC dataset (e.g., MH01, MH02, etc.)",
    )

    parser.add_argument(
        "--associations",
        help="Path to associations file (required for RGB-D examples)",
    )

    parser.add_argument(
        "--vocab",
        help="Path to vocabulary file (default: auto-detected)",
    )

    parser.add_argument(
        "--settings-file",
        help="Path to settings YAML file (default: auto-detected)",
    )

    parser.add_argument(
        "--output-dir",
        default="benchmark_results",
        help="Output directory for results (default: benchmark_results)",
    )

    parser.add_argument(
        "--orbslam-root",
        help="Path to ORB-SLAM2 installation (default: $ORBSLAM2_ROOT or /opt/orb_slam2)",
    )

    return parser.parse_args()


def main():
    """Main entry point"""
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        args = parse_arguments()

        # Validate example-specific requirements
        if args.example in ["mono_euroc", "stereo_euroc"] and not args.settings:
            logger.error("--settings is required for EuRoC examples")
            sys.exit(1)

        if args.example == "rgbd_tum" and not args.associations:
            logger.error("--associations is required for RGB-D TUM example")
            sys.exit(1)

        # Create runner
        runner = ORBSLAM2Runner(
            output_dir=args.output_dir, orbslam_root=args.orbslam_root
        )

        # Get timestamps path for EuRoC examples
        time_path = None
        if args.example in ["mono_euroc", "stereo_euroc"]:
            time_path = str(runner.get_timestamps_path(args.settings))

        # Run benchmark
        logger.info("=" * 60)
        logger.info("Starting ORB-SLAM2 Benchmark")
        logger.info("=" * 60)

        result = runner.run_example(
            example_name=args.example,
            dataset_path=args.dataset,
            associations_path=args.associations,
            time_path=time_path,
            vocab_path=args.vocab,
            settings_path=args.settings_file,
        )

        # Print summary
        logger.info("")
        logger.info("=" * 60)
        logger.info("BENCHMARK SUMMARY")
        logger.info("=" * 60)
        logger.info(f"Example:      {result['example_name']}")
        logger.info(f"Duration:     {result['duration_seconds']:.2f} seconds")
        logger.info(f"Return Code:  {result['return_code']}")

        if result["cpu_stats"]:
            logger.info(
                f"CPU Usage:    Mean={result['cpu_stats']['mean']:.1f}%, Max={result['cpu_stats']['max']:.1f}%"
            )
            logger.info(
                f"Memory Usage: Mean={result['memory_stats']['mean_mb']:.1f} MB, Max={result['memory_stats']['max_mb']:.1f} MB"
            )

        logger.info(f"Results:      {result['output_dir']}")
        logger.info("=" * 60)

        # Exit with appropriate code
        if result["return_code"] == 0:
            logger.info("Benchmark completed successfully")
            sys.exit(0)
        else:
            logger.error("Benchmark failed")
            sys.exit(1)

    except ConfigurationError as e:
        logger.error(f"Configuration error: {e}")
        sys.exit(2)
    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
        sys.exit(130)
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        sys.exit(3)


if __name__ == "__main__":
    main()
