"""
Purpose
-------
Visualize ROS node log data from lemniscate trajectory experiments conducted
on 2025/11/05. The ROS node logged yaw, previous yaw, ground angular velocity,
and estimated slip at each timestep. This script parses those logs and produces
three plots per experiment: slip over time, yaw over time, and yaw difference
(heading change per step).

Three experiments are available:
  - experiment_inside_05.11.25   (indoor run 1)
  - experiment_inside_2_05.11.25 (indoor run 2)
  - experiment_outside_05.11.25  (outdoor run)

Usage
-----
Set SELECTED_LOG to one of the log filenames above to process a single
experiment, or to None to process all three.
"""

import re
import math
import matplotlib.pyplot as plt


LOG_FILES = [
    "experiment_inside_05.11.25",
    "experiment_inside_2_05.11.25",
    "experiment_outside_05.11.25",
]

# Set to a filename from LOG_FILES to run one experiment, or None to run all.
SELECTED_LOG = "experiment_inside_05.11.25"


# --- Regex patterns (same format for all log files) ---
pattern_yaw     = re.compile(r"Yaw:\s*([-+]?\d*\.\d+|\d+)")
pattern_yaw_prev = re.compile(r"Yaw previous:\s*([-+]?\d*\.\d+|\d+|None)")
pattern_ang_vel = re.compile(r"Ground angular velocity z:\s*([-+]?\d*\.\d+|\d+)")
pattern_slip    = re.compile(r"slip:\s*([-+]?\d*\.\d+|\d+)")


def parse_log(log_file):
    yaw_list      = []
    yaw_prev_list = []
    ang_vel_list  = []
    slip_list     = []

    with open(log_file, "r") as f:
        for line in f:
            if "Yaw:" in line and "Ground angular velocity z" in line:
                yaw_match      = pattern_yaw.search(line)
                yaw_prev_match = pattern_yaw_prev.search(line)
                ang_vel_match  = pattern_ang_vel.search(line)
                if yaw_match and yaw_prev_match and ang_vel_match:
                    yaw_prev_str = yaw_prev_match.group(1)
                    yaw_list.append(float(yaw_match.group(1)))
                    yaw_prev_list.append(float(yaw_prev_str) if yaw_prev_str != "None" else None)
                    ang_vel_list.append(float(ang_vel_match.group(1)))

            if "slip:" in line:
                slip_match = pattern_slip.search(line)
                if slip_match:
                    slip_list.append(float(slip_match.group(1)))

    return yaw_list, yaw_prev_list, ang_vel_list, slip_list


def plot_experiment(log_file):
    print(f"\nProcessing: {log_file}")
    yaw_list, yaw_prev_list, ang_vel_list, slip_list = parse_log(log_file)
    print(f"  Extracted {len(yaw_list)} yaw values, {len(slip_list)} slip values")

    plt.figure(figsize=(6, 4))
    plt.plot(slip_list, label='Slip')
    plt.xlabel('Time Step')
    plt.ylabel('Slip')
    plt.title(f'Slip Over Time — {log_file}')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(6, 4))
    plt.plot(yaw_list, label='Yaw')
    plt.xlabel('Time Step')
    plt.ylabel('Yaw (radians)')
    plt.title(f'Yaw Over Time — {log_file}')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    yaw_diff = []
    for i in range(len(yaw_list)):
        if yaw_prev_list[i] is not None:
            diff = yaw_list[i] - yaw_prev_list[i]
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            yaw_diff.append(diff)
        else:
            yaw_diff.append(0.0)

    plt.figure(figsize=(6, 4))
    plt.plot(yaw_diff, label='Yaw Difference')
    plt.xlabel('Time Step')
    plt.ylabel('Yaw Difference (radians)')
    plt.title(f'Yaw Difference Over Time — {log_file}')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()


# --- Entry point ---

if SELECTED_LOG is not None:
    plot_experiment(SELECTED_LOG)
else:
    for log_file in LOG_FILES:
        plot_experiment(log_file)
