#!/usr/bin/env python3
import math
import yaml
import matplotlib.pyplot as plt

def load_yaml_data(yaml_path):
    """Load YAML file and return its contents."""
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        return data
    except Exception as e:
        print(f"Error loading YAML file from {yaml_path}: {e}")
        return None

def get_time_axis(data_section):
    """
    Returns a time axis.
    If the data section has a 'time' key, that data is returned.
    Otherwise, generate time from indices.
    """
    if "time" in data_section:
        return data_section["time"]
    else:
        n = len(data_section["pos"])  # assume same length for all entries
        return list(range(n))

def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def get_closest_index(time_list, t):
    """Return the index of the entry in time_list closest to time t."""
    if not time_list:
        return None
    closest_index = 0
    min_diff = abs(time_list[0] - t)
    for i, time_val in enumerate(time_list):
        diff = abs(time_val - t)
        if diff < min_diff:
            min_diff = diff
            closest_index = i
    return closest_index

def compute_position_errors_sync(gt_time, gt_pos, ref_time, ref_pos):
    """For each ground truth timestamp, find the closest reference data and compute Euclidean position error."""
    errors = []
    for i, t in enumerate(gt_time):
        idx = get_closest_index(ref_time, t)
        if idx is not None:
            dx = gt_pos[i][0] - ref_pos[idx][0]
            dy = gt_pos[i][1] - ref_pos[idx][1]
            errors.append(math.sqrt(dx*dx + dy*dy))
    return errors

def compute_yaw_errors_sync(gt_time, gt_yaw, ref_time, ref_yaw):
    """For each ground truth timestamp, find the closest reference data and compute absolute yaw error (wrapped)."""
    errors = []
    for i, t in enumerate(gt_time):
        idx = get_closest_index(ref_time, t)
        if idx is not None:
            diff = gt_yaw[i] - ref_yaw[idx]
            diff = math.atan2(math.sin(diff), math.cos(diff))
            errors.append(abs(diff))
    return errors

def main():
    # --- Load ground truth YAML file ---
    gt_yaml_path = "/home/sunny/FRA532_LAB1/src/ackermann_controller/yaml/lab1.2/stan-0.25.yaml"
    gt_data_full = load_yaml_data(gt_yaml_path)
    if gt_data_full is None:
        return

    if "ground_truth" not in gt_data_full:
        print("Error: YAML file does not contain 'ground_truth' section.")
        return

    gt_data = gt_data_full["ground_truth"]
    gt_pos = gt_data["pos"]
    gt_yaw = gt_data["yaw"]
    gt_time = get_time_axis(gt_data)

    print(f"Loaded ground truth data with {len(gt_pos)} position entries and {len(gt_yaw)} yaw entries.")

    # Trim the first 50 entries from ground truth data
    trim_count = 50
    gt_pos = gt_pos[trim_count:]
    gt_yaw = gt_yaw[trim_count:]
    gt_time = gt_time[trim_count:]
    print(f"After trimming, ground truth data has {len(gt_pos)} position entries.")

    # Extract ground truth X and Y positions
    gt_x = [p[0] for p in gt_pos]
    gt_y = [p[1] for p in gt_pos]

    # --- Load reference YAML file ---
    ref_yaml_path = "/home/sunny/FRA532_LAB1/src/ackermann_controller/yaml/path.yaml"
    ref_data = load_yaml_data(ref_yaml_path)
    if ref_data is None:
        return
    # Assume the reference file is a list of dictionaries with x, y, and yaw fields.
    ref_x = [entry["x"] for entry in ref_data]
    ref_y = [entry["y"] for entry in ref_data]
    ref_yaw = [entry["yaw"] for entry in ref_data]
    
    # Map reference yaw values to the range [-pi, pi]
    ref_yaw = [wrap_angle(yaw) for yaw in ref_yaw]
    
    # Since the reference file doesn't have time, generate a time axis based on the ground truth time range.
    if len(gt_time) > 1 and len(ref_data) > 1:
        start = gt_time[0]
        end = gt_time[-1]
        n = len(ref_data)
        interval = (end - start) / (n - 1)
        ref_time = [start + i * interval for i in range(n)]
    else:
        # Fall back to simple indexing if not enough information.
        ref_time = list(range(len(ref_data)))
    
    print(f"Loaded reference data with {len(ref_data)} entries.")

    # --- Create a figure with 2 subplots: Position and Yaw ---
    fig_data, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    # Position Trajectories
    ax1.plot(gt_x, gt_y, 'bo-', markersize=1, label='Path Tracking Position')
    ax1.plot(ref_x, ref_y, '--r', markersize=3, label='Reference Position')
    ax1.set_title("Position Trajectory: Path Tracking vs Reference")
    ax1.set_xlabel("X Position")
    ax1.set_ylabel("Y Position")
    ax1.legend()
    ax1.grid(True)
    
    # Yaw vs Time
    ax2.plot(gt_time, gt_yaw, 'bo-', markersize=1, label='Path Tracking Yaw')
    ax2.plot(ref_time, ref_yaw, '--r', markersize=3, label='Reference Yaw')
    ax2.set_title("Yaw: Path Tracking vs Reference")
    ax2.set_xlabel("Time (s) / Index")
    ax2.set_ylabel("Yaw (rad)")
    ax2.legend()
    ax2.grid(True)
    
    # --- Compute Errors ---
    # Combine reference positions into a list of [x, y] pairs.
    ref_pos = list(zip(ref_x, ref_y))
    pos_errors = compute_position_errors_sync(gt_time, gt_pos, ref_time, ref_pos)
    yaw_errors = compute_yaw_errors_sync(gt_time, gt_yaw, ref_time, ref_yaw)

    # --- Create a figure with 2 subplots for Error Distributions ---
    fig_error, (ax3, ax4) = plt.subplots(1, 2, figsize=(14, 6))
    # Position Error Distribution
    ax3.hist(pos_errors, bins=50, color='g', alpha=0.7, label='Position Error')
    ax3.set_title("Position Error Distribution")
    ax3.set_xlabel("Error (m)")
    ax3.set_ylabel("Frequency")
    ax3.legend()
    ax3.grid(True)
    
    # Yaw Error Distribution
    ax4.hist(yaw_errors, bins=50, color='m', alpha=0.7, label='Yaw Error')
    ax4.set_title("Yaw Error Distribution")
    ax4.set_xlabel("Error (rad)")
    ax4.set_ylabel("Frequency")
    ax4.legend()
    ax4.grid(True)
    
    # Show the figures.
    plt.show()

if __name__ == '__main__':
    main()
